#!/usr/bin/env python3
# test_slam_manager.py — unit tests for T06: SlamManagerNode (mocked ROS2)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from unittest.mock import MagicMock, patch
import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import String


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    from dome_nav.slam_manager_node import SlamManagerNode
    n = SlamManagerNode()
    yield n
    n.destroy_node()


# --- on_map tests ---

def test_on_map_sets_map_ready(node):
    assert not node.map_ready
    msg = OccupancyGrid()
    msg.info = MapMetaData()
    node.on_map(msg)
    assert node.map_ready


def test_on_map_publishes_mapping_status(node):
    published = []
    node.status_pub.publish = lambda m: published.append(m.data)

    msg = OccupancyGrid()
    msg.info = MapMetaData()
    node.on_map(msg)

    assert len(published) == 1
    assert published[0] == "mapping"


def test_on_map_publishes_every_call(node):
    published = []
    node.status_pub.publish = lambda m: published.append(m.data)

    msg = OccupancyGrid()
    msg.info = MapMetaData()
    node.on_map(msg)
    node.on_map(msg)

    assert len(published) == 2
    assert all(s == "mapping" for s in published)


# --- save_map tests ---

def test_save_map_returns_false_when_service_unavailable(node):
    node.serialize_client.wait_for_service = MagicMock(return_value=False)
    result = node.save_map()
    assert result is False


def test_save_map_calls_service_with_correct_path(node):
    mock_future = MagicMock()
    node.serialize_client.wait_for_service = MagicMock(return_value=True)
    node.serialize_client.call_async = MagicMock(return_value=mock_future)

    result = node.save_map()

    call_args = node.serialize_client.call_async.call_args[0][0]
    assert call_args.filename == node.map_persist_path
    assert result is True


def test_on_save_done_logs_error_on_future_none(node):
    mock_future = MagicMock()
    mock_future.result.return_value = None

    with patch.object(node, "get_logger") as mock_logger:
        node._on_save_done(mock_future)
        mock_logger().error.assert_called_once()


def test_save_map_creates_directory(node, tmp_path):
    new_path = str(tmp_path / "subdir" / "slam_map")
    node.map_persist_path = new_path

    mock_future = MagicMock()
    node.serialize_client.wait_for_service = MagicMock(return_value=True)
    node.serialize_client.call_async = MagicMock(return_value=mock_future)

    node.save_map()

    assert (tmp_path / "subdir").exists()
