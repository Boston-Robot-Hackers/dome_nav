#!/usr/bin/env python3
# explore_manager_node.py — autonomous frontier exploration via Nav2
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import functools
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import tf2_ros

from dome_nav.explore_telemetry import TelemetryWriter
from dome_nav.frontier_explorer import (
    MapInfo,
    find_frontier_clusters,
    pick_best_frontier,
)

XY = tuple[float, float]

GOAL_STATUS_NAMES = {4: "succeeded", 5: "canceled", 6: "aborted"}


class ExploreManagerNode(Node):
    MIN_FRONTIER_SIZE = 10
    BLACKLIST_RADIUS = 0.5
    # nudged goal = frontier_dist - GOAL_INSET_M; must exceed xy_goal_tolerance (0.5m)
    MIN_FRONTIER_DIST = 2.0
    # pull goal off frontier boundary to avoid Nav2 worldToMap boundary errors
    GOAL_INSET_M = 0.3
    EXPLORE_HZ = 2.0
    NO_FRONTIER_PATIENCE = 8
    # cancel goal to avoid blocking during Nav2 BT recovery loops
    GOAL_TIMEOUT_S = 25.0

    def __init__(self):
        super().__init__("explore_manager_node")
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, "/explore/status", 10)
        self.intent_sub = self.create_subscription(
            String, "/intent", self.on_intent, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.on_map, 10
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / self.EXPLORE_HZ, self.explore_tick)

        self.declare_parameter("max_explore_radius", 0.0)
        self.declare_parameter("map_name", "unknown")
        self.max_explore_radius: float = self.get_parameter("max_explore_radius").value
        self.map_name: str = self.get_parameter("map_name").value
        self.telemetry = TelemetryWriter(self.map_name, self.get_logger().info)

        self.state = "idle"
        self.latest_map: OccupancyGrid | None = None
        self.active_goal = False
        self.goal_handle = None
        self.blacklist: set[XY] = set()
        self.start_xy: XY | None = None
        self.no_frontier_count = 0
        self.goal_start_time: float | None = None
        self.current_goal_centroid: XY | None = None
        self.current_goal_xy: XY | None = None
        self.goal_count = 0
        self.goals_reached = 0
        self.goals_failed = 0
        self.get_logger().info("ExploreManagerNode ready.")

    def on_map(self, msg: OccupancyGrid):
        # Called on every /map update from slam_toolbox (typically 1 Hz).
        self.latest_map = msg

    def on_intent(self, msg: String):
        # Called on every /intent message. Only acts on exploration_start (when
        # idle/done) and exploration_stop. All other intents are silently ignored.
        try:
            intent = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(
                f"Malformed intent JSON on /intent: {msg.data!r}"
            )
            return
        name = intent.get("name", "")
        if name == "exploration_start" and self.state in ("idle", "done"):
            self.blacklist.clear()
            self.no_frontier_count = 0
            self.goal_count = 0
            self.goals_reached = 0
            self.goals_failed = 0
            self.start_xy = self.robot_xy_in_map()
            self.state = "exploring"
            self.publish_status("exploring")
            r = (
                f", max_radius={self.max_explore_radius}m"
                if self.max_explore_radius > 0 else ""
            )
            self.get_logger().info(f"Exploration started{r}.")
            self.telemetry.write(
                "session_start", map_name=self.map_name,
                start_xy=list(self.start_xy) if self.start_xy else None,
                params={
                    "min_frontier_dist": self.MIN_FRONTIER_DIST,
                    "goal_inset": self.GOAL_INSET_M,
                    "timeout_s": self.GOAL_TIMEOUT_S,
                    "max_radius": self.max_explore_radius,
                },
            )
        elif name == "exploration_stop":
            self.stop_exploring("idle")

    def explore_tick(self):
        # Timer callback at EXPLORE_HZ. Skips if not exploring or a goal is active.
        # One goal at a time: next frontier only sent after previous goal completes.
        self.publish_status(self.state)
        if self.state != "exploring":
            return
        if self.active_goal:
            self.check_goal_timeout()
            return
        self.find_and_send_frontier()

    def check_goal_timeout(self):
        # Cancels the active goal if it has exceeded GOAL_TIMEOUT_S.
        if self.goal_start_time is None:
            return
        if (time.monotonic() - self.goal_start_time) <= self.GOAL_TIMEOUT_S:
            return
        elapsed = round(time.monotonic() - self.goal_start_time, 1)
        self.get_logger().warning(
            f"Goal timed out after {elapsed}s — cancelling and blacklisting."
        )
        self.telemetry.write(
            "goal_result", goal_num=self.goal_count, status="timeout",
            elapsed_s=elapsed, robot_xy=None, blacklisted=len(self.blacklist),
            goal_xy=(
                [round(self.current_goal_xy[0], 3), round(self.current_goal_xy[1], 3)]
                if self.current_goal_xy else None
            ),
        )
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        if self.current_goal_centroid is not None:
            self.blacklist.add(self.current_goal_centroid)
        self.active_goal = False
        self.goal_start_time = None
        self.current_goal_centroid = None
        self.current_goal_xy = None

    def find_and_send_frontier(self):
        # Core exploration step: scan latest map for frontier clusters, pick the
        # nearest valid one, nudge it inward, and send a Nav2 goal. If no valid
        # frontier exists for NO_FRONTIER_PATIENCE consecutive ticks, declares done.
        if self.latest_map is None:
            return
        robot_xy = self.robot_xy_in_map()
        if robot_xy is None:
            self.get_logger().warning("TF map→base_footprint unavailable — waiting.")
            return
        m = self.latest_map
        info = MapInfo(
            width=m.info.width, height=m.info.height, resolution=m.info.resolution,
            origin_x=m.info.origin.position.x, origin_y=m.info.origin.position.y,
        )
        clusters = find_frontier_clusters(list(m.data), info)
        target = pick_best_frontier(
            clusters, info, robot_xy,
            self.MIN_FRONTIER_SIZE, self.blacklist, self.BLACKLIST_RADIUS,
            self.max_explore_radius, self.start_xy, self.MIN_FRONTIER_DIST,
        )
        if target is None:
            self.no_frontier_count += 1
            self.get_logger().info(
                f"No frontiers found "
                f"(tick {self.no_frontier_count}/{self.NO_FRONTIER_PATIENCE})."
            )
            self.telemetry.write(
                "no_frontier", tick=self.no_frontier_count,
                patience=self.NO_FRONTIER_PATIENCE, blacklisted=len(self.blacklist),
            )
            if self.no_frontier_count >= self.NO_FRONTIER_PATIENCE:
                self.get_logger().info(
                    "Frontier patience exhausted — exploration done."
                )
                self.telemetry.write(
                    "session_end", reason="done", goals_sent=self.goal_count,
                    reached=self.goals_reached, failed=self.goals_failed,
                )
                self.state = "done"
                self.publish_status("done")
            return
        self.no_frontier_count = 0
        self.send_nav_goal(self.nudge_toward_robot(target, robot_xy), centroid=target)

    def nudge_toward_robot(self, xy: XY, robot_xy: XY) -> XY:
        # Pulls frontier centroid GOAL_INSET_M toward robot so the nav goal lands
        # inside the costmap rather than on the unknown-cell boundary (which causes
        # worldToMap out-of-bounds errors in Nav2).
        dx = robot_xy[0] - xy[0]
        dy = robot_xy[1] - xy[1]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < self.GOAL_INSET_M:
            return xy
        scale = self.GOAL_INSET_M / dist
        return (xy[0] + dx * scale, xy[1] + dy * scale)

    def send_nav_goal(self, xy: XY, centroid: XY):
        # Sends NavigateToPose goal (nudged xy) to Nav2. Centroid is stored
        # separately for blacklisting — the comparison must use the original centroid,
        # not the nudged goal, so blacklist lookups stay consistent.
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
        self.goal_start_time = time.monotonic()
        self.current_goal_centroid = centroid
        self.current_goal_xy = xy
        self.goal_count += 1
        robot_xy = self.robot_xy_in_map()
        dist = (
            math.sqrt((xy[0] - robot_xy[0]) ** 2 + (xy[1] - robot_xy[1]) ** 2)
            if robot_xy else -1.0
        )
        self.get_logger().info(
            f"Goal #{self.goal_count}: ({xy[0]:.2f},{xy[1]:.2f})"
            f" dist={dist:.2f}m blacklisted={len(self.blacklist)}"
        )
        self.telemetry.write(
            "goal_sent", goal_num=self.goal_count,
            goal_xy=[round(xy[0], 3), round(xy[1], 3)],
            frontier_xy=[round(centroid[0], 3), round(centroid[1], 3)],
            dist_m=round(dist, 3),
            robot_xy=[round(v, 3) for v in robot_xy] if robot_xy else None,
            blacklisted=len(self.blacklist),
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            functools.partial(self.on_goal_accepted, xy=xy, centroid=centroid)
        )

    def on_goal_accepted(self, future, xy: XY, centroid: XY):
        # Nav2 action callback: goal either accepted (attach result callback) or
        # rejected (blacklist centroid so we don't retry the same unreachable spot).
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning(
                f"Goal rejected at ({xy[0]:.2f}, {xy[1]:.2f}) — blacklisting centroid."
            )
            self.blacklist.add(centroid)
            self.active_goal = False
            return
        self.goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self.on_goal_result, xy=xy, centroid=centroid)
        )

    def on_goal_result(self, future, xy: XY, centroid: XY):
        # Called when Nav2 finishes (success or failure). Always blacklists centroid
        # to prevent re-visiting the same frontier on the next tick.
        self.goal_handle = None
        self.active_goal = False
        elapsed = (
            round(time.monotonic() - self.goal_start_time, 1)
            if self.goal_start_time else 0.0
        )
        self.goal_start_time = None
        self.current_goal_centroid = None
        self.current_goal_xy = None
        result = future.result()
        robot_xy = self.robot_xy_in_map()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.goals_reached += 1
            status_name = "reached"
            self.get_logger().info(
                f"Goal #{self.goal_count} REACHED"
                f" ({xy[0]:.2f},{xy[1]:.2f}) in {elapsed}s."
            )
        else:
            self.goals_failed += 1
            status_name = GOAL_STATUS_NAMES.get(result.status, str(result.status))
            self.get_logger().warning(
                f"Goal #{self.goal_count} FAILED ({xy[0]:.2f},{xy[1]:.2f})"
                f" status={status_name} after {elapsed}s — blacklisting."
            )
        self.telemetry.write(
            "goal_result", goal_num=self.goal_count,
            goal_xy=[round(xy[0], 3), round(xy[1], 3)],
            status=status_name, elapsed_s=elapsed,
            robot_xy=[round(v, 3) for v in robot_xy] if robot_xy else None,
            blacklisted=len(self.blacklist),
        )
        # blacklist centroid (not nudged goal) so pick_best_frontier comparison is exact
        self.blacklist.add(centroid)

    def stop_exploring(self, new_state: str):
        # Cancels any active Nav2 goal and transitions state. Called by
        # exploration_stop intent (→ "idle") or patience exhausted (→ "done").
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.active_goal = False
        self.state = new_state
        self.publish_status(new_state)
        self.get_logger().info(f"Exploration stopped → {new_state}.")
        self.telemetry.write(
            "session_end", reason=new_state, goals_sent=self.goal_count,
            reached=self.goals_reached, failed=self.goals_failed,
        )

    def robot_xy_in_map(self) -> XY | None:
        # Looks up map→base_footprint TF. Returns None if transform not yet available.
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            t = tf.transform.translation
            return (t.x, t.y)
        except (
            tf2_ros.LookupException,
            tf2_ros.ExtrapolationException,
            tf2_ros.ConnectivityException,
        ):
            return None

    def publish_status(self, status: str):
        # Publishes JSON to /explore/status at 2Hz.
        robot_xy = self.robot_xy_in_map()
        data: dict = {"state": status}
        has_goal = self.current_goal_xy is not None and robot_xy is not None
        is_active = status == "exploring" and has_goal
        if is_active:
            gx, gy = self.current_goal_xy
            dist = math.sqrt((gx - robot_xy[0]) ** 2 + (gy - robot_xy[1]) ** 2)
            data["goal_num"] = self.goal_count
            data["goal_xy"] = [round(gx, 2), round(gy, 2)]
            data["dist_m"] = round(dist, 2)
            data["elapsed_s"] = (
                round(time.monotonic() - self.goal_start_time, 1)
                if self.goal_start_time else None
            )
            data["blacklisted"] = len(self.blacklist)
        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ExploreManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.telemetry.write(
            "session_end", reason="shutdown", goals_sent=node.goal_count,
            reached=node.goals_reached, failed=node.goals_failed,
        )
        node.telemetry.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
