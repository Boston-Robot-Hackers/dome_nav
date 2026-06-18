#!/usr/bin/env python3
# test_map_validation.py — manual integration tests for T02: slam_toolbox map and TF
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
# Requires live dome_nav stack: bl dome_nav robot.launch.py

import time

import pytest
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import tf2_ros


pytestmark = pytest.mark.manual


class MapListener(Node):
    def __init__(self):
        super().__init__("test_map_listener")
        self.msg = None
        self.sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, 10)

    def on_map(self, msg: OccupancyGrid):
        self.msg = msg

    def wait_for_map(self, timeout_sec: float = 10.0) -> OccupancyGrid | None:
        deadline = time.time() + timeout_sec
        while time.time() < deadline and self.msg is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.msg


@pytest.fixture(scope="module")
def ros_node():
    rclpy.init()
    node = MapListener()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_map_published(ros_node):
    msg = ros_node.wait_for_map(timeout_sec=10.0)
    assert msg is not None, "/map not published within 10s — is stack running?"


def test_map_metadata(ros_node):
    msg = ros_node.wait_for_map(timeout_sec=10.0)
    assert msg.info.resolution > 0.0
    assert msg.info.width > 0
    assert msg.info.height > 0


def test_map_has_free_and_occupied_cells(ros_node):
    msg = ros_node.wait_for_map(timeout_sec=10.0)
    cells = list(msg.data)
    assert 0 in cells, "No free cells (0) in map — LiDAR may not be scanning"
    assert 100 in cells, "No occupied cells (100) in map — walls not detected"


def test_map_to_odom_tf(ros_node):
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, ros_node)
    deadline = time.time() + 10.0
    transform = None
    while time.time() < deadline:
        try:
            transform = tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
            break
        except tf2_ros.LookupException:
            rclpy.spin_once(ros_node, timeout_sec=0.2)
    assert transform is not None, "map→odom TF not available within 10s"
