#!/usr/bin/env python3
# test_explorer_manager_node.py — unit tests for ExplorerManagerNode
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
from dome_nav.explore_context import ExploreParams, GoalDecision, GoalOutcome


class MockAlgorithm:
    # latest_clusters/latest_diag are still read by the node's telemetry path
    # (removed from the protocol in F23 T02); kept here so the stub satisfies it.
    latest_clusters = []
    latest_diag = None

    def __init__(self, decision=None):
        # Default: a benign block so no-op ticks debounce without crashing.
        self.decision = decision if decision is not None else GoalDecision.blocked()

    def next_goal(self, ctx):
        return self.decision


@pytest.fixture(scope="module")
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    from dome_nav.explorer_manager_node import ExplorerManagerNode
    with patch("tf2_ros.TransformListener"), \
         patch("rclpy.action.ActionClient"), \
         patch("dome_nav.explorer_manager_node.TelemetryWriter",
               return_value=MagicMock()):
        n = ExplorerManagerNode(algorithm=MockAlgorithm())
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


def test_find_frontier_blocked_increments_count(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_frontier_count = 0
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.blocked())
    node.find_and_send_frontier()
    assert node.no_frontier_count == 1
    node.send_nav_goal.assert_not_called()


def test_find_frontier_explored_done_ends_session_immediately(node):
    # EXPLORED_DONE ends the session at once — no NO_FRONTIER_PATIENCE wait, and
    # WITHOUT the node reading latest_clusters to decide.
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_frontier_count = 0
    node.send_nav_goal = MagicMock()
    node.dump_frontier_exhaustion = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.done())
    node.find_and_send_frontier()
    assert node.state == "done"
    node.send_nav_goal.assert_not_called()


def test_find_frontier_blocked_patience_clears_blacklist_once(node):
    # First patience exhaustion on a block clears the blacklist and retries —
    # it does NOT declare done.
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_frontier_count = node.NO_FRONTIER_PATIENCE - 1
    node.blacklist = {(1.0, 1.0)}
    node.blacklist_cleared_once = False
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.blocked())
    node.find_and_send_frontier()
    assert node.state == "exploring"
    assert node.blacklist == set()
    assert node.blacklist_cleared_once is True
    assert node.no_frontier_count == 0


def test_find_frontier_blocked_patience_after_clear_sets_done(node):
    # A second patience exhaustion (blacklist already cleared) gives up → done.
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_frontier_count = node.NO_FRONTIER_PATIENCE - 1
    node.blacklist_cleared_once = True
    node.send_nav_goal = MagicMock()
    node.dump_frontier_exhaustion = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.blocked())
    node.find_and_send_frontier()
    assert node.state == "done"
    node.send_nav_goal.assert_not_called()


def test_find_frontier_found_resets_patience_count(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_frontier_count = 5
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.new_goal((3.0, 0.0)))
    node.find_and_send_frontier()
    assert node.no_frontier_count == 0
    node.send_nav_goal.assert_called_once()


def test_find_frontier_sends_algorithm_goal(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.new_goal((1.0, 2.0)))
    node.find_and_send_frontier()
    call_args = node.send_nav_goal.call_args
    sent_xy = call_args[0][0]
    assert sent_xy == (1.0, 2.0)


# --- check_goal_timeout ---

def test_timeout_not_expired_does_nothing(node):
    node.has_active_goal = True
    node.goal_start_time = time.monotonic()
    node.goal_handle = MagicMock()
    node.current_goal_xy = (0.7, 0.0)
    node.check_goal_timeout()
    node.goal_handle.cancel_goal_async.assert_not_called()
    assert node.has_active_goal is True


def test_timeout_expired_cancels_goal(node):
    node.has_active_goal = True
    node.goal_start_time = 0.0  # ancient → always expired
    mock_handle = MagicMock()
    node.goal_handle = mock_handle
    node.current_goal_xy = (4.7, 5.0)
    node.check_goal_timeout()
    mock_handle.cancel_goal_async.assert_called_once()


def test_timeout_expired_blacklists_goal(node):
    node.has_active_goal = True
    node.goal_start_time = 0.0
    node.goal_handle = MagicMock()
    node.current_goal_xy = (4.7, 5.0)
    node.blacklist = set()
    node.check_goal_timeout()
    assert (4.7, 5.0) in node.blacklist


def test_timeout_expired_clears_active_state(node):
    node.has_active_goal = True
    node.goal_start_time = 0.0
    node.goal_handle = MagicMock()
    node.current_goal_xy = (0.7, 0.0)
    node.check_goal_timeout()
    assert node.has_active_goal is False
    assert node.goal_start_time is None
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


# --- min_frontier_size ROS parameter wiring ---

def test_min_frontier_size_default_matches_explore_params(node):
    assert node.min_frontier_size == ExploreParams().min_frontier_size


def test_min_frontier_size_plumbed_into_params(node):
    assert node.params.min_frontier_size == node.min_frontier_size


# --- min_frontier_dist ROS parameter wiring ---

def test_min_frontier_dist_default_matches_explore_params(node):
    # Default parameter must match the dataclass so real-robot behavior is unchanged.
    assert node.min_frontier_dist == ExploreParams().min_frontier_dist


def test_min_frontier_dist_plumbed_into_params(node):
    assert node.params.min_frontier_dist == node.min_frontier_dist


# --- frontier_buffer_cells ROS parameter wiring ---

def test_frontier_buffer_cells_default_matches_explore_params(node):
    assert node.frontier_buffer_cells == ExploreParams().frontier_buffer_cells


def test_frontier_buffer_cells_plumbed_into_params(node):
    assert node.params.frontier_buffer_cells == node.frontier_buffer_cells


# --- goal_in_global_costmap bounds check (worldToMap guard) ---

def costmap_2m(resolution=0.05):
    # 40x40 cell costmap (2m x 2m) with origin at (0,0), all free.
    cm = OccupancyGrid()
    cm.info.resolution = resolution
    cm.info.width = 40
    cm.info.height = 40
    cm.info.origin.position.x = 0.0
    cm.info.origin.position.y = 0.0
    cm.data = [0] * (40 * 40)
    return cm


def test_goal_in_costmap_true_when_no_costmap_yet(node):
    # Startup must not be blocked before the first costmap arrives.
    node.latest_global_costmap = None
    assert node.goal_in_global_costmap((5.0, 5.0)) is True


def test_goal_in_costmap_true_for_interior_goal(node):
    node.latest_global_costmap = costmap_2m()
    assert node.goal_in_global_costmap((1.0, 1.0)) is True


def test_goal_in_costmap_false_for_goal_past_edge(node):
    # 2m-wide costmap; a goal at x=2.05m maps one cell past the east edge, the
    # exact worldToMap failure that aborted planning with PLAN/NO_VALID_PATH.
    node.latest_global_costmap = costmap_2m()
    assert node.goal_in_global_costmap((2.05, 1.0)) is False
    assert node.goal_in_global_costmap((1.0, -0.1)) is False
