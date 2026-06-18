#!/usr/bin/env python3
# slam_manager_node.py — lifecycle node: watches /map, persists slam_toolbox pose graph
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SerializePoseGraph
from dome_nav.utils import dome_home


def default_map_path() -> str:
    return os.path.join(dome_home(), "slam_map")


class SlamManagerNode(LifecycleNode):
    SAVE_PERIOD_SEC = 30.0

    def __init__(self):
        super().__init__("slam_manager_node")
        self.declare_parameter("map_persist_path", default_map_path())
        self.map_persist_path = (
            self.get_parameter("map_persist_path").get_parameter_value().string_value
        )
        self.map_ready = False
        self.map_sub = None
        self.status_pub = None
        self.serialize_client = None
        self.save_timer = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)
        self.status_pub = self.create_lifecycle_publisher(
            String, "/dome_nav/slam_status", 10
        )
        self.serialize_client = self.create_client(
            SerializePoseGraph, "/slam_toolbox/serialize_map"
        )
        self.get_logger().info(f"Configured. path={self.map_persist_path}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.save_timer = self.create_timer(self.SAVE_PERIOD_SEC, self.periodic_save)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if self.save_timer is not None:
            self.destroy_timer(self.save_timer)
            self.save_timer = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_entities()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        # Synchronous final save: spin the service call to completion here, before the
        # node is destroyed. The old plain-Node main() fired this async after spin()
        # had already returned, so the callback never ran and the map was lost (I01).
        if self.map_ready:
            self.save_map_sync()
        self.destroy_entities()
        return TransitionCallbackReturn.SUCCESS

    def destroy_entities(self):
        if self.save_timer is not None:
            self.destroy_timer(self.save_timer)
            self.save_timer = None
        if self.map_sub is not None:
            self.destroy_subscription(self.map_sub)
            self.map_sub = None
        if self.status_pub is not None:
            self.destroy_lifecycle_publisher(self.status_pub)
            self.status_pub = None

    def ensure_map_dir(self):
        parent = os.path.dirname(self.map_persist_path)
        if parent:
            os.makedirs(parent, exist_ok=True)

    def on_map(self, msg: OccupancyGrid):
        first_map = not self.map_ready
        if first_map:
            self.map_ready = True
            self.get_logger().info("Map received — slam_toolbox is mapping.")
        status = String()
        status.data = "mapping"
        self.status_pub.publish(status)
        if first_map:
            self.save_map_async()

    def periodic_save(self):
        if self.map_ready:
            self.save_map_async()

    def save_map_async(self):
        if not self.prepare_save():
            return
        future = self.serialize_client.call_async(self.serialize_request())
        future.add_done_callback(self.on_save_done)

    def save_map_sync(self):
        if not self.prepare_save():
            return
        future = self.serialize_client.call_async(self.serialize_request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.on_save_done(future)

    def prepare_save(self) -> bool:
        self.ensure_map_dir()
        if not self.serialize_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning("serialize_map service unavailable — not saved.")
            return False
        return True

    def serialize_request(self) -> SerializePoseGraph.Request:
        req = SerializePoseGraph.Request()
        req.filename = self.map_persist_path
        return req

    def on_save_done(self, future):
        if future.result() is not None:
            self.get_logger().info(f"Pose graph saved to {self.map_persist_path}")
        else:
            self.get_logger().error("Failed to serialize pose graph.")


def main():
    rclpy.init()
    node = SlamManagerNode()
    node.trigger_configure()
    node.trigger_activate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.trigger_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
