#!/usr/bin/env python3
# test_pluggable_explore_manager_node.py — unit tests for PluggableExploreManagerNode
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import math
import time
from unittest.mock import MagicMock, patch
import pytest
import rclpy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from dome_nav.explore_context import ExploreParams


class MockAlgorithm:
    latest_clusters = []
    latest_diag = None

    def __init__(self, return_value=None):
        self.return_value = return_value

    def next_goal(self, ctx):
        return self.return_value


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    from dome_nav.pluggable_explore_manager_node import PluggableExploreManagerNode
    with patch("tf2_ros.TransformListener"), \
         patch("rclpy.action.ActionClient"), \
         patch("dome_nav.pluggable_explore_manager_node.TelemetryWriter",
               return_value=MagicMock()):
        n = PluggableExploreManagerNode(algorithm=MockAlgorithm())
    yield n
    n.destroy_node()


def make_map():
    return OccupancyGrid()


def make_intent(name):
    msg = String()
    msg.data = json.dumps({"name": name, "source": "cli", "slots": {}})
    return msg


# --- on_intent state transitions ---

def test_intent_start_from_idle(node):
    node.state = "idle"
    node.robot_xy_in_map = MagicMock(return_value=(1.0, 2.0))
    node.on_intent(make_intent("exploration_start"))
    assert node.state == "exploring"


def test_intent_start_from_done(node):
    node.state = "done"
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.on_intent(make_intent("exploration_start"))
    assert node.state == "exploring"


def test_intent_start_while_exploring_ignored(node):
    node.state = "exploring"
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.goal_count = 3
    node.on_intent(make_intent("exploration_start"))
    assert node.state == "exploring"
    assert node.goal_count == 3  # not reset


def test_intent_stop_sets_idle(node):
    node.state = "exploring"
    node.goal_handle = None
    node.on_intent(make_intent("exploration_stop"))
    assert node.state == "idle"


def test_intent_malformed_json_no_crash(node):
    msg = String()
    msg.data = "not json {"
    node.on_intent(msg)  # must not raise


def test_intent_unknown_name_no_state_change(node):
    node.state = "idle"
    node.on_intent(make_intent("navigation_go"))
    assert node.state == "idle"


def test_intent_start_resets_blacklist(node):
    node.state = "idle"
    node.blacklist = {(1.0, 2.0), (3.0, 4.0)}
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.on_intent(make_intent("exploration_start"))
    assert node.blacklist == set()


def test_intent_start_resets_counters(node):
    node.state = "idle"
    node.goal_count = 5
    node.goals_reached = 3
    node.goals_failed = 2
    node.no_frontier_count = 4
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.on_intent(make_intent("exploration_start"))
    assert node.goal_count == 0
    assert node.goals_reached == 0
    assert node.goals_failed == 0
    assert node.no_frontier_count == 0


# --- find_and_send_frontier via MockAlgorithm ---

def test_find_frontier_no_map_early_return(node):
    node.state = "exploring"
    node.latest_map = None
    node.send_nav_goal = MagicMock()
    node.find_and_send_frontier()
    node.send_nav_goal.assert_not_called()


def test_find_frontier_no_robot_xy_early_return(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=None)
    node.send_nav_goal = MagicMock()
    node.find_and_send_frontier()
    node.send_nav_goal.assert_not_called()


def test_find_frontier_no_target_increments_count(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_frontier_count = 0
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(return_value=None)
    node.find_and_send_frontier()
    assert node.no_frontier_count == 1
    node.send_nav_goal.assert_not_called()


def test_find_frontier_patience_exhausted_sets_done(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_frontier_count = node.NO_FRONTIER_PATIENCE - 1
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(return_value=None)
    node.find_and_send_frontier()
    assert node.state == "done"
    node.send_nav_goal.assert_not_called()


def test_find_frontier_found_resets_patience_count(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_frontier_count = 5
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(return_value=(3.0, 0.0))
    node.find_and_send_frontier()
    assert node.no_frontier_count == 0
    node.send_nav_goal.assert_called_once()


def test_find_frontier_sends_algorithm_goal(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(return_value=(1.0, 2.0))
    node.find_and_send_frontier()
    call_args = node.send_nav_goal.call_args
    sent_xy = call_args[0][0]
    assert sent_xy == (1.0, 2.0)


# --- check_goal_redirect disabled under prefer_farthest ---

def test_redirect_fires_when_not_prefer_farthest(node):
    node.prefer_farthest = False
    node.is_redirecting = False
    node.latest_map = MagicMock()
    node.current_goal_xy = (0.0, 0.0)
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.frontier_goal_for_current_map = MagicMock(return_value=(5.0, 0.0))
    node.goal_handle = MagicMock()
    node.check_goal_redirect()
    node.goal_handle.cancel_goal_async.assert_called_once()
    assert node.is_redirecting is True


def test_redirect_suppressed_when_prefer_farthest(node):
    node.prefer_farthest = True
    node.is_redirecting = False
    node.latest_map = MagicMock()
    node.current_goal_xy = (0.0, 0.0)
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.frontier_goal_for_current_map = MagicMock(return_value=(5.0, 0.0))
    node.goal_handle = MagicMock()
    node.check_goal_redirect()
    node.goal_handle.cancel_goal_async.assert_not_called()
    assert node.is_redirecting is False


# --- check_goal_timeout ---

def test_timeout_not_expired_does_nothing(node):
    node.has_active_goal = True
    node.goal_start_time = time.monotonic()
    node.goal_handle = MagicMock()
    node.current_goal_centroid = (1.0, 0.0)
    node.current_goal_xy = (0.7, 0.0)
    node.check_goal_timeout()
    node.goal_handle.cancel_goal_async.assert_not_called()
    assert node.has_active_goal is True


def test_timeout_expired_cancels_goal(node):
    node.has_active_goal = True
    node.goal_start_time = 0.0  # ancient → always expired
    mock_handle = MagicMock()
    node.goal_handle = mock_handle
    node.current_goal_centroid = (5.0, 5.0)
    node.current_goal_xy = (4.7, 5.0)
    node.check_goal_timeout()
    mock_handle.cancel_goal_async.assert_called_once()


def test_timeout_expired_blacklists_centroid(node):
    node.has_active_goal = True
    node.goal_start_time = 0.0
    node.goal_handle = MagicMock()
    node.current_goal_centroid = (5.0, 5.0)
    node.current_goal_xy = (4.7, 5.0)
    node.blacklist = set()
    node.check_goal_timeout()
    assert (5.0, 5.0) in node.blacklist


def test_timeout_expired_clears_active_state(node):
    node.has_active_goal = True
    node.goal_start_time = 0.0
    node.goal_handle = MagicMock()
    node.current_goal_centroid = (1.0, 0.0)
    node.current_goal_xy = (0.7, 0.0)
    node.check_goal_timeout()
    assert node.has_active_goal is False
    assert node.goal_start_time is None
    assert node.current_goal_centroid is None
    assert node.current_goal_xy is None


def test_timeout_no_start_time_does_nothing(node):
    node.goal_start_time = None
    node.goal_handle = MagicMock()
    node.check_goal_timeout()
    node.goal_handle.cancel_goal_async.assert_not_called()


# --- publish_status JSON shape ---

def test_publish_status_idle_json(node):
    node.goals_reached = 0
    node.goals_failed = 0
    node.robot_xy_in_map = MagicMock(return_value=None)
    published = []
    node.status_pub.publish = lambda m: published.append(json.loads(m.data))
    node.publish_status("idle")
    assert published == [{"state": "idle", "reached": 0, "failed": 0}]


def test_publish_status_done_carries_counters(node):
    node.goals_reached = 5
    node.goals_failed = 1
    node.robot_xy_in_map = MagicMock(return_value=None)
    published = []
    node.status_pub.publish = lambda m: published.append(json.loads(m.data))
    node.publish_status("done")
    assert published[0] == {"state": "done", "reached": 5, "failed": 1}


def test_publish_status_exploring_no_goal(node):
    node.current_goal_xy = None
    node.goals_reached = 1
    node.goals_failed = 0
    node.goal_count = 2
    node.blacklist = set()
    node.no_frontier_count = 3
    node.robot_xy_in_map = MagicMock(return_value=(1.0, 2.0))
    published = []
    node.status_pub.publish = lambda m: published.append(json.loads(m.data))
    node.publish_status("exploring")
    d = published[0]
    assert d["state"] == "exploring"
    assert d["reached"] == 1
    assert d["goal_num"] == 2
    assert d["no_frontier_ticks"] == 3
    assert "goal_xy" not in d
    assert "dist_m" not in d


def test_publish_status_exploring_with_goal_fields(node):
    node.current_goal_xy = (3.0, 4.0)
    node.goals_reached = 2
    node.goals_failed = 0
    node.goal_count = 3
    node.goal_start_time = time.monotonic() - 5.0
    node.blacklist = {(1.0, 0.0), (2.0, 0.0)}
    node.no_frontier_count = 0
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    published = []
    node.status_pub.publish = lambda m: published.append(json.loads(m.data))
    node.publish_status("exploring")
    d = published[0]
    assert d["state"] == "exploring"
    assert d["reached"] == 2
    assert d["failed"] == 0
    assert d["goal_num"] == 3
    assert d["goal_xy"] == [3.0, 4.0]
    assert d["blacklisted"] == 2
    assert d["no_frontier_ticks"] == 0
    assert abs(d["dist_m"] - round(math.sqrt(9 + 16), 2)) < 1e-6


def test_publish_status_dist_correct(node):
    node.current_goal_xy = (3.0, 0.0)
    node.goals_reached = 0
    node.goals_failed = 0
    node.goal_count = 1
    node.goal_start_time = time.monotonic()
    node.blacklist = set()
    node.no_frontier_count = 0
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    published = []
    node.status_pub.publish = lambda m: published.append(json.loads(m.data))
    node.publish_status("exploring")
    assert published[0]["dist_m"] == 3.0


# --- default parameters must not form an empty [min, max] frontier-distance band ---

def test_default_max_frontier_dist_exceeds_min_frontier_dist(node):
    assert node.max_frontier_dist > ExploreParams().min_frontier_dist
