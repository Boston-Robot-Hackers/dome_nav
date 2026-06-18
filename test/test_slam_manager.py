#!/usr/bin/env python3
# test_slam_manager.py — unit tests for SlamManagerNode lifecycle (mock ROS2)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from unittest.mock import MagicMock, patch
import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    from dome_nav.slam_manager_node import SlamManagerNode
    n = SlamManagerNode()
    n.trigger_configure()
    yield n
    n.destroy_node()


# --- lifecycle transitions ---

def test_configure_creates_entities(node):
    assert node.map_sub is not None
    assert node.status_pub is not None
    assert node.serialize_client is not None


def test_activate_starts_save_timer(node):
    assert node.save_timer is None
    node.trigger_activate()
    assert node.save_timer is not None


def test_deactivate_stops_save_timer(node):
    node.trigger_activate()
    node.trigger_deactivate()
    assert node.save_timer is None


# --- on_map ---

def test_on_map_sets_map_ready(node):
    assert not node.map_ready
    node.on_map(OccupancyGrid())
    assert node.map_ready


def test_on_map_publishes_mapping_status(node):
    published = []
    node.status_pub.publish = lambda m: published.append(m.data)
    node.on_map(OccupancyGrid())
    assert published == ["mapping"]


# --- save ---

def test_prepare_save_false_when_service_unavailable(node):
    node.serialize_client.wait_for_service = MagicMock(return_value=False)
    assert node.prepare_save() is False


def test_save_map_async_calls_service_with_correct_path(node):
    node.serialize_client.wait_for_service = MagicMock(return_value=True)
    node.serialize_client.call_async = MagicMock(return_value=MagicMock())
    node.save_map_async()
    req = node.serialize_client.call_async.call_args[0][0]
    assert req.filename == node.map_persist_path


def test_save_map_creates_directory(node, tmp_path):
    node.map_persist_path = str(tmp_path / "subdir" / "slam_map")
    node.serialize_client.wait_for_service = MagicMock(return_value=True)
    node.serialize_client.call_async = MagicMock(return_value=MagicMock())
    node.save_map_async()
    assert (tmp_path / "subdir").exists()


def test_on_save_done_logs_error_on_future_none(node):
    future = MagicMock()
    future.result.return_value = None
    with patch.object(node, "get_logger") as mock_logger:
        node.on_save_done(future)
        mock_logger().error.assert_called_once()


# --- shutdown persistence: regression for I01 (map must be saved on shutdown) ---

def test_shutdown_saves_when_map_ready(node):
    node.map_ready = True
    node.save_map_sync = MagicMock()
    node.trigger_shutdown()
    node.save_map_sync.assert_called_once()


def test_shutdown_skips_save_when_no_map(node):
    node.map_ready = False
    node.save_map_sync = MagicMock()
    node.trigger_shutdown()
    node.save_map_sync.assert_not_called()
