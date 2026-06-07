#!/usr/bin/env python3
# slam_manager_node.py — monitors slam_toolbox state and manages map persistence
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SerializePoseGraph
from dome_nav.utils import dome_home


def default_map_path() -> str:
    return os.path.join(dome_home(), "slam_map")


class SlamManagerNode(Node):
    def __init__(self):
        super().__init__("slam_manager_node")
        self.declare_parameter("map_persist_path", default_map_path())

        self.map_persist_path = self.get_parameter("map_persist_path").get_parameter_value().string_value

        self.map_ready = False

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.status_pub = self.create_publisher(String, "/dome_nav/slam_status", 10)

        self.serialize_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

        self.save_timer = self.create_timer(30.0, self.periodic_save)

        self.get_logger().info(f"SlamManagerNode ready. map_persist_path={self.map_persist_path}")

    def periodic_save(self):
        if self.map_ready:
            self.save_map()

    def on_map(self, msg: OccupancyGrid):
        if not self.map_ready:
            self.map_ready = True
            self.get_logger().info("Map received — slam_toolbox is mapping.")
        status = String()
        status.data = "mapping" if self.map_ready else "waiting"
        self.status_pub.publish(status)

    def save_map(self):
        os.makedirs(os.path.dirname(self.map_persist_path), exist_ok=True)

        if not self.serialize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("serialize_map service not available — map not saved.")
            return False

        req = SerializePoseGraph.Request()
        req.filename = self.map_persist_path
        future = self.serialize_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            self.get_logger().info(f"Pose graph saved to {self.map_persist_path}")
            return True
        self.get_logger().error("Failed to serialize pose graph.")
        return False


def main():
    rclpy.init()
    node = SlamManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.save_map()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
