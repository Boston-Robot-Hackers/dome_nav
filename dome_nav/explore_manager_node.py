#!/usr/bin/env python3
# explore_manager_node.py — autonomous frontier exploration via Nav2
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import functools
import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import tf2_ros

from dome_nav.frontier_explorer import MapInfo, find_frontier_clusters, pick_best_frontier


class ExploreManagerNode(Node):
    MIN_FRONTIER_SIZE = 10
    BLACKLIST_RADIUS = 0.5
    MIN_FRONTIER_DIST = 0.5   # skip frontiers closer than this to robot — within goal tolerance, robot won't move
    GOAL_INSET_M = 0.6        # pull goal inward so frontier-edge centroid lands inside costmap bounds
    EXPLORE_HZ = 2.0
    NO_FRONTIER_PATIENCE = 8  # consecutive no-frontier ticks before declaring done

    def __init__(self):
        super().__init__("explore_manager_node")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, "/explore/status", 10)
        self.intent_sub = self.create_subscription(String, "/intent", self.on_intent, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.EXPLORE_HZ, self.explore_tick)

        self.declare_parameter("max_explore_radius", 0.0)
        self.max_explore_radius: float = self.get_parameter("max_explore_radius").value

        self.state = "idle"
        self.latest_map: OccupancyGrid | None = None
        self.active_goal = False
        self.goal_handle = None
        self.blacklist: set[tuple[float, float]] = set()
        self.start_xy: tuple[float, float] | None = None
        self.no_frontier_count = 0
        self.get_logger().info("ExploreManagerNode ready.")

    def on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def on_intent(self, msg: String):
        try:
            intent = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Malformed intent JSON on /intent: {msg.data!r}")
            return
        name = intent.get("name", "")
        if name == "exploration_start" and self.state in ("idle", "done"):
            self.blacklist.clear()
            self.no_frontier_count = 0
            self.start_xy = self.robot_xy_in_map()
            self.state = "exploring"
            self.publish_status("exploring")
            radius_msg = f", max_radius={self.max_explore_radius}m" if self.max_explore_radius > 0 else ""
            self.get_logger().info(f"Exploration started{radius_msg}.")
        elif name == "exploration_stop":
            self.stop_exploring("idle")

    def explore_tick(self):
        if self.state != "exploring" or self.active_goal:
            return
        self.find_and_send_frontier()

    def find_and_send_frontier(self):
        if self.latest_map is None:
            return
        robot_xy = self.robot_xy_in_map()
        if robot_xy is None:
            self.get_logger().warning("TF map→base_footprint unavailable — waiting.")
            return
        m = self.latest_map
        info = MapInfo(
            width=m.info.width,
            height=m.info.height,
            resolution=m.info.resolution,
            origin_x=m.info.origin.position.x,
            origin_y=m.info.origin.position.y,
        )
        clusters = find_frontier_clusters(list(m.data), info)
        target = pick_best_frontier(
            clusters, info, robot_xy,
            self.MIN_FRONTIER_SIZE, self.blacklist, self.BLACKLIST_RADIUS,
            self.max_explore_radius, self.start_xy,
            self.MIN_FRONTIER_DIST,
        )
        if target is None:
            self.no_frontier_count += 1
            self.get_logger().info(
                f"No frontiers found (tick {self.no_frontier_count}/{self.NO_FRONTIER_PATIENCE})."
            )
            if self.no_frontier_count >= self.NO_FRONTIER_PATIENCE:
                self.get_logger().info("Frontier patience exhausted — exploration done.")
                self.state = "done"
                self.publish_status("done")
            return
        self.no_frontier_count = 0
        self.send_nav_goal(self.nudge_toward_robot(target, robot_xy), centroid=target)

    def nudge_toward_robot(self, xy: tuple[float, float], robot_xy: tuple[float, float]) -> tuple[float, float]:
        dx = robot_xy[0] - xy[0]
        dy = robot_xy[1] - xy[1]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < self.GOAL_INSET_M:
            return xy
        scale = self.GOAL_INSET_M / dist
        return (xy[0] + dx * scale, xy[1] + dy * scale)

    def send_nav_goal(self, xy: tuple[float, float], centroid: tuple[float, float]):
        if not self.nav_client.server_is_ready():
            self.get_logger().warning("NavigateToPose server not ready — will retry.")
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = xy[0]
        goal.pose.pose.position.y = xy[1]
        goal.pose.pose.orientation.w = 1.0
        self.active_goal = True
        self.get_logger().info(
            f"Sending frontier goal: ({xy[0]:.2f}, {xy[1]:.2f}), centroid: ({centroid[0]:.2f}, {centroid[1]:.2f})"
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(functools.partial(self.on_goal_accepted, xy=xy, centroid=centroid))

    def on_goal_accepted(self, future, xy: tuple[float, float], centroid: tuple[float, float]):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning(f"Goal rejected at ({xy[0]:.2f}, {xy[1]:.2f}) — blacklisting centroid.")
            self.blacklist.add(centroid)
            self.active_goal = False
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(functools.partial(self.on_goal_result, xy=xy, centroid=centroid))

    def on_goal_result(self, future, xy: tuple[float, float], centroid: tuple[float, float]):
        self.goal_handle = None
        self.active_goal = False
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached frontier ({xy[0]:.2f}, {xy[1]:.2f}).")
        else:
            self.get_logger().warning(f"Failed to reach ({xy[0]:.2f}, {xy[1]:.2f}) — blacklisting centroid.")
        # blacklist centroid (not nudged goal) so pick_best_frontier comparison is exact
        self.blacklist.add(centroid)

    def stop_exploring(self, new_state: str):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.active_goal = False
        self.state = new_state
        self.publish_status(new_state)
        self.get_logger().info(f"Exploration stopped → {new_state}.")

    def robot_xy_in_map(self) -> tuple[float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ExploreManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
