#!/usr/bin/env python3
# nav_manager_node.py — translates dome intents into Nav2 navigation goals
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavManagerNode(Node):
    def __init__(self):
        super().__init__("nav_manager_node")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.status_pub = self.create_publisher(String, "/dome_nav/nav_status", 10)

        self.intent_sub = self.create_subscription(String, "/intent", self.on_intent, 10)
        self.targets_sub = self.create_subscription(String, "/targets/confirmed", self.on_targets, 10)

        self.confirmed_targets: list[dict] = []
        self.get_logger().info("NavManagerNode ready.")

    def on_targets(self, msg: String):
        try:
            self.confirmed_targets = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Could not parse /targets/confirmed JSON.")

    def on_intent(self, msg: String):
        try:
            intent = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        action = intent.get("action", "")
        if action == "go_to_object":
            label = intent.get("label", "")
            self.navigate_to_object(label)
        elif action == "cancel_navigation":
            self.cancel_navigation()

    def navigate_to_object(self, label: str):
        target = self.find_nearest_confirmed(label)
        if target is None:
            self.get_logger().warning(f"No confirmed target found for label={label!r}.")
            self.publish_status(f"no_target:{label}")
            return

        xyz = target.get("xyz_world", [0.0, 0.0, 0.0])
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(xyz[0])
        goal_pose.pose.position.y = float(xyz[1])
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available.")
            self.publish_status("nav_unavailable")
            return

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        self.get_logger().info(f"Navigating to {label} at {xyz}.")
        self.publish_status(f"navigating:{label}")
        self.nav_client.send_goal_async(goal, feedback_callback=self.on_nav_feedback)

    def cancel_navigation(self):
        if self.nav_client.server_is_ready():
            self.nav_client._cancel_goal_async()
            self.publish_status("cancelled")

    def on_nav_feedback(self, feedback_msg):
        pass

    def find_nearest_confirmed(self, label: str) -> dict | None:
        matches = [t for t in self.confirmed_targets if t.get("label") == label]
        if not matches:
            return None
        return matches[0]

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
