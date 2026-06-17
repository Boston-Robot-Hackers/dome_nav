#!/usr/bin/env python3
# slam_manager_node.py — monitors slam_toolbox state and manages map persistence
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SerializePoseGraph
from dome_nav.utils import dome_home
from dome_nav.slam_manager import SlamManager
import os


def default_map_path() -> str:
    return os.path.join(dome_home(), "slam_map")


class SlamManagerNode(Node):
    def __init__(self):
        super().__init__("slam_manager_node")
        self.declare_parameter("map_persist_path", default_map_path())

        _path = self.get_parameter("map_persist_path").get_parameter_value().string_value
        self._manager = SlamManager(_path)

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.status_pub = self.create_publisher(String, "/dome_nav/slam_status", 10)

        self.serialize_client = self.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map")

        self.save_timer = self.create_timer(30.0, self.periodic_save)

        self.get_logger().info(f"SlamManagerNode ready. map_persist_path={self._manager.map_persist_path}")

    @property
    def map_persist_path(self) -> str:
        return self._manager.map_persist_path

    @map_persist_path.setter
    def map_persist_path(self, value: str):
        self._manager.map_persist_path = value

    @property
    def map_ready(self) -> bool:
        return self._manager.map_ready

    def periodic_save(self):
        if self._manager.should_save():
            self.save_map()

    def on_map(self, msg: OccupancyGrid):
        was_ready = self._manager.map_ready
        status_str = self._manager.on_map_received()
        if not was_ready:
            self.get_logger().info("Map received — slam_toolbox is mapping.")
        status = String()
        status.data = status_str
        self.status_pub.publish(status)

    def save_map(self):
        self._manager.ensure_map_dir()

        if not self.serialize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("serialize_map service not available — map not saved.")
            return False

        req = SerializePoseGraph.Request()
        req.filename = self.map_persist_path
        future = self.serialize_client.call_async(req)
        future.add_done_callback(self._on_save_done)
        return True

    def _on_save_done(self, future):
        if future.result() is not None:
            self.get_logger().info(f"Pose graph saved to {self.map_persist_path}")
        else:
            self.get_logger().error("Failed to serialize pose graph.")


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
