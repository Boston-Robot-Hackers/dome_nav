#!/usr/bin/env python3
# nav_manager_node.py — translates dome intents into Nav2 navigation goals
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import functools
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros
from dome_nav.nav_manager import NavManager


class NavManagerNode(Node):
    def __init__(self):
        super().__init__("nav_manager_node")

        self.manager = NavManager()

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, "/dome_nav/nav_status", 10)

        self.loc_status_pub = self.create_publisher(String, "/dome_nav/localization_status", 10)
        self.loc_score_pub = self.create_publisher(Float32, "/dome_nav/localization_score", 10)

        self.intent_sub = self.create_subscription(String, "/intent", self.on_intent, 10)
        self.targets_sub = self.create_subscription(String, "/targets/confirmed", self.on_targets, 10)
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, amcl_qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.goal_handle = None
        self.last_loc_status = "localizing"
        self.last_loc_score = 0.0
        self.create_timer(1.0, self.publish_localization)
        self.get_logger().info("NavManagerNode ready.")

    def on_targets(self, msg: String):
        if not self.manager.on_targets(msg.data):
            self.get_logger().warning("Could not parse /targets/confirmed JSON.")

    def on_intent(self, msg: String):
        result = self.manager.parse_intent(msg.data)
        if result is None:
            self.get_logger().warning(f"Malformed or unknown intent: {msg.data!r}")
            return
        action, intent = result
        if action == "navigation_go":
            label = intent.get("slots", {}).get("label", "")
            self.navigate_to_object(label)
        elif action == "navigation_cancel":
            self.navigation_cancel()

    def navigate_to_object(self, label: str):
        target = self.find_nearest_confirmed(label)
        if target is None:
            self.get_logger().warning(f"No confirmed target found for label={label!r}.")
            self.publish_status(self.manager.navigate_status(label, None))
            return

        xyz = target["xyz_world"]  # validated at ingest in on_targets
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = xyz[0]
        goal_pose.pose.position.y = xyz[1]
        goal_pose.pose.position.z = 0.0
        yaw = target.get("yaw_world", 0.0)
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available.")
            self.publish_status("nav_unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.get_logger().info(f"Navigating to {label} at {xyz}.")
        self.publish_status(self.manager.navigate_status(label, target))
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(functools.partial(self.on_goal_accepted, label=label))

    def on_goal_accepted(self, future, label: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected by Nav2.")
            self.publish_status(f"goal_rejected:{label}")
            return
        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(functools.partial(self.on_goal_result, label=label))

    def on_goal_result(self, future, label: str):
        self.goal_handle = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(f"done:{label}")
        else:
            self.publish_status(f"failed:{label}")

    def navigation_cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.publish_status("cancelled")

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        status, score = self.manager.check_localization(list(msg.pose.covariance))
        self.last_loc_status = status
        self.last_loc_score = score
        self.publish_localization()

    def publish_localization(self):
        s_msg = String()
        s_msg.data = self.last_loc_status
        self.loc_status_pub.publish(s_msg)
        f_msg = Float32()
        f_msg.data = float(self.last_loc_score)
        self.loc_score_pub.publish(f_msg)

    def robot_xy_in_map(self) -> tuple[float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            return None

    def find_nearest_confirmed(self, label: str) -> dict | None:
        robot_xy = self.robot_xy_in_map()
        if robot_xy is None:
            self.get_logger().warning("map→base_footprint TF unavailable — returning first match.")
        return self.manager.find_nearest_confirmed(label, robot_xy)

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavManagerNode()
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
