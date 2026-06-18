#!/usr/bin/env python3
# test_nav_manager.py — unit tests for NavManagerNode (mocked ROS2)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
from unittest.mock import MagicMock, patch, call
import pytest
import rclpy
from action_msgs.msg import GoalStatus


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    from dome_nav.nav_manager_node import NavManagerNode
    with patch("tf2_ros.TransformListener"):
        n = NavManagerNode()
    yield n
    n.destroy_node()


# --- find_nearest_confirmed tests ---

def test_find_nearest_no_matches(node):
    node._manager.confirmed_targets = [{"label": "chair", "xyz_world": [1.0, 0.0, 0.0]}]
    assert node.find_nearest_confirmed("table") is None


def test_find_nearest_empty_targets(node):
    node._manager.confirmed_targets = []
    assert node.find_nearest_confirmed("chair") is None


def test_find_nearest_single_match(node):
    import tf2_ros
    node._manager.confirmed_targets = [{"label": "chair", "xyz_world": [3.0, 4.0, 0.0]}]
    node.tf_buffer.lookup_transform = MagicMock(
        side_effect=tf2_ros.LookupException("no tf")
    )
    result = node.find_nearest_confirmed("chair")
    assert result is not None
    assert result["xyz_world"] == [3.0, 4.0, 0.0]


def test_find_nearest_returns_closest(node):
    node._manager.confirmed_targets = [
        {"label": "chair", "xyz_world": [10.0, 0.0, 0.0]},
        {"label": "chair", "xyz_world": [1.0, 0.0, 0.0]},
        {"label": "chair", "xyz_world": [5.0, 0.0, 0.0]},
    ]
    mock_tf = MagicMock()
    mock_tf.transform.translation.x = 0.0
    mock_tf.transform.translation.y = 0.0
    node.tf_buffer.lookup_transform = MagicMock(return_value=mock_tf)

    result = node.find_nearest_confirmed("chair")
    assert result["xyz_world"] == [1.0, 0.0, 0.0]


def test_find_nearest_closest_from_non_origin(node):
    node._manager.confirmed_targets = [
        {"label": "box", "xyz_world": [0.0, 0.0, 0.0]},
        {"label": "box", "xyz_world": [8.0, 0.0, 0.0]},
    ]
    mock_tf = MagicMock()
    mock_tf.transform.translation.x = 7.0
    mock_tf.transform.translation.y = 0.0
    node.tf_buffer.lookup_transform = MagicMock(return_value=mock_tf)

    result = node.find_nearest_confirmed("box")
    assert result["xyz_world"] == [8.0, 0.0, 0.0]


def test_find_nearest_tf_unavailable_returns_first(node):
    import tf2_ros
    node._manager.confirmed_targets = [
        {"label": "cup", "xyz_world": [10.0, 0.0, 0.0]},
        {"label": "cup", "xyz_world": [1.0, 0.0, 0.0]},
    ]
    node.tf_buffer.lookup_transform = MagicMock(
        side_effect=tf2_ros.LookupException("no tf")
    )
    result = node.find_nearest_confirmed("cup")
    assert result["xyz_world"] == [10.0, 0.0, 0.0]


# --- on_intent routing tests ---

def test_on_intent_go_to_object_calls_navigate(node):
    node.navigate_to_object = MagicMock()
    msg = MagicMock()
    msg.data = json.dumps({"action": "go_to_object", "label": "chair"})
    node.on_intent(msg)
    node.navigate_to_object.assert_called_once_with("chair")


def test_on_intent_cancel_calls_cancel(node):
    node.cancel_navigation = MagicMock()
    msg = MagicMock()
    msg.data = json.dumps({"action": "cancel_navigation"})
    node.on_intent(msg)
    node.cancel_navigation.assert_called_once()


def test_on_intent_invalid_json_ignored(node):
    node.navigate_to_object = MagicMock()
    msg = MagicMock()
    msg.data = "not json"
    node.on_intent(msg)
    node.navigate_to_object.assert_not_called()


# --- navigate_to_object tests ---

def test_navigate_no_target_publishes_no_target(node):
    node._manager.confirmed_targets = []
    node.publish_status = MagicMock()
    node.navigate_to_object("ghost")
    node.publish_status.assert_called_once_with("no_target:ghost")


def test_navigate_server_unavailable_publishes_nav_unavailable(node):
    node._manager.confirmed_targets = [{"label": "chair", "xyz_world": [1.0, 0.0, 0.0]}]
    node.publish_status = MagicMock()
    node.nav_client.wait_for_server = MagicMock(return_value=False)
    node.navigate_to_object("chair")
    node.publish_status.assert_called_with("nav_unavailable")


def test_navigate_sends_goal_and_publishes_navigating(node):
    node._manager.confirmed_targets = [{"label": "chair", "xyz_world": [2.0, 3.0, 0.0]}]
    node.publish_status = MagicMock()
    node.nav_client.wait_for_server = MagicMock(return_value=True)
    mock_future = MagicMock()
    node.nav_client.send_goal_async = MagicMock(return_value=mock_future)

    node.navigate_to_object("chair")

    node.nav_client.send_goal_async.assert_called_once()
    node.publish_status.assert_called_with("navigating:chair")


# --- cancel_navigation tests ---

def test_cancel_with_no_goal_handle_does_nothing(node):
    node._goal_handle = None
    node.publish_status = MagicMock()
    node.cancel_navigation()
    node.publish_status.assert_not_called()


def test_cancel_calls_goal_handle_cancel(node):
    mock_handle = MagicMock()
    node._goal_handle = mock_handle
    node.publish_status = MagicMock()
    node.cancel_navigation()
    mock_handle.cancel_goal_async.assert_called_once()
    node.publish_status.assert_called_once_with("cancelled")
    assert node._goal_handle is None


# --- goal result callback tests ---

def test_on_goal_result_success_publishes_done(node):
    node.publish_status = MagicMock()
    node._goal_handle = MagicMock()
    mock_future = MagicMock()
    mock_future.result.return_value.status = GoalStatus.STATUS_SUCCEEDED
    node._on_goal_result(mock_future, label="chair")
    node.publish_status.assert_called_once_with("done:chair")
    assert node._goal_handle is None


def test_on_goal_result_aborted_publishes_failed(node):
    node.publish_status = MagicMock()
    node._goal_handle = MagicMock()
    mock_future = MagicMock()
    mock_future.result.return_value.status = GoalStatus.STATUS_ABORTED
    node._on_goal_result(mock_future, label="chair")
    node.publish_status.assert_called_once_with("failed:chair")


def test_on_goal_accepted_rejected_publishes_goal_rejected(node):
    node.publish_status = MagicMock()
    mock_future = MagicMock()
    mock_future.result.return_value.accepted = False
    node._on_goal_accepted(mock_future, label="table")
    node.publish_status.assert_called_once_with("goal_rejected:table")


def test_on_goal_accepted_stores_handle_and_registers_result_cb(node):
    node.publish_status = MagicMock()
    mock_handle = MagicMock()
    mock_handle.accepted = True
    mock_result_future = MagicMock()
    mock_handle.get_result_async.return_value = mock_result_future
    mock_future = MagicMock()
    mock_future.result.return_value = mock_handle

    node._on_goal_accepted(mock_future, label="box")

    assert node._goal_handle is mock_handle
    mock_result_future.add_done_callback.assert_called_once()
