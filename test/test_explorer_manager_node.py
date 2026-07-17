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
from dome_nav.frontier_params import FrontierParams
from dome_nav.frontier_algorithm import FrontierAlgorithm


class MockAlgorithm:
    # Minimal stub: implements next_goal + declare_params only. It exposes NO
    # clusters, no diag, and none of the optional render/diagnostics hooks —
    # proving the node no longer requires latest_clusters/latest_diag or any
    # visualization surface (F23 T02). frontier_params is None: it needs only the
    # shared params and declares no frontier ROS params of its own (F23 T03).
    frontier_params = None

    def __init__(self, decision=None):
        # Default: a benign block so no-op ticks debounce without crashing.
        self.decision = decision if decision is not None else GoalDecision.blocked()

    def declare_params(self, node):
        pass

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


@pytest.fixture
def frontier_node(ros):
    # Node running the default FrontierAlgorithm, which self-declares its frontier
    # ROS params in the node's namespace (F23 T03).
    from dome_nav.explorer_manager_node import ExplorerManagerNode
    with patch("tf2_ros.TransformListener"), \
         patch("rclpy.action.ActionClient"), \
         patch("dome_nav.explorer_manager_node.TelemetryWriter",
               return_value=MagicMock()):
        n = ExplorerManagerNode(algorithm=FrontierAlgorithm())
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
    node.no_target_count = 4
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.on_intent(make_intent("exploration_start"))
    assert node.goal_count == 0
    assert node.goals_reached == 0
    assert node.goals_failed == 0
    assert node.no_target_count == 0


# --- find_and_send_goal via MockAlgorithm ---

def test_find_frontier_no_map_early_return(node):
    node.state = "exploring"
    node.latest_map = None
    node.send_nav_goal = MagicMock()
    node.find_and_send_goal()
    node.send_nav_goal.assert_not_called()


def test_find_frontier_no_robot_xy_early_return(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=None)
    node.send_nav_goal = MagicMock()
    node.find_and_send_goal()
    node.send_nav_goal.assert_not_called()


def test_find_frontier_blocked_increments_count(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = 0
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.blocked())
    node.find_and_send_goal()
    assert node.no_target_count == 1
    node.send_nav_goal.assert_not_called()


def test_find_frontier_explored_done_ends_session_immediately(node):
    # EXPLORED_DONE ends the session at once — no NO_TARGET_PATIENCE wait, and
    # WITHOUT the node reading latest_clusters to decide.
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = 0
    node.send_nav_goal = MagicMock()
    node.dump_exhaustion = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.done())
    node.find_and_send_goal()
    assert node.state == "done"
    node.send_nav_goal.assert_not_called()


def test_find_frontier_blocked_patience_clears_blacklist_once(node):
    # First patience exhaustion on a block clears the blacklist and retries —
    # it does NOT declare done.
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = node.NO_TARGET_PATIENCE - 1
    node.blacklist = {(1.0, 1.0)}
    node.blacklist_cleared_once = False
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.blocked())
    node.find_and_send_goal()
    assert node.state == "exploring"
    assert node.blacklist == set()
    assert node.blacklist_cleared_once is True
    assert node.no_target_count == 0


def test_find_frontier_blocked_patience_after_clear_sets_done(node):
    # A second patience exhaustion (blacklist already cleared) gives up → done.
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = node.NO_TARGET_PATIENCE - 1
    node.blacklist_cleared_once = True
    node.send_nav_goal = MagicMock()
    node.dump_exhaustion = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.blocked())
    node.find_and_send_goal()
    assert node.state == "done"
    node.send_nav_goal.assert_not_called()


def test_find_frontier_found_resets_patience_count(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = 5
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.new_goal((3.0, 0.0)))
    node.find_and_send_goal()
    assert node.no_target_count == 0
    node.send_nav_goal.assert_called_once()


def test_find_frontier_sends_algorithm_goal(node):
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.new_goal((1.0, 2.0)))
    node.find_and_send_goal()
    call_args = node.send_nav_goal.call_args
    sent_xy = call_args[0][0]
    assert sent_xy == (1.0, 2.0)


# --- F23 T02: visualization + diagnostics off the protocol ---
# The MockAlgorithm above exposes ONLY next_goal — no latest_clusters,
# latest_diag, or any render/diagnostics hook. These assert the node runs its
# visualization and telemetry paths against such a stub without error.

def test_protocol_no_longer_requires_cluster_state():
    # The required protocol surface must not mention frontier internals.
    from dome_nav.explore_context import ExplorationAlgorithm
    annotations = getattr(ExplorationAlgorithm, "__annotations__", {})
    assert "latest_clusters" not in annotations
    assert "latest_diag" not in annotations


def test_publish_markers_no_hook_does_not_publish(node):
    # Stub has no render_markers hook -> nothing published, no error.
    node.marker_pub.publish = MagicMock()
    node.publish_markers()
    node.marker_pub.publish.assert_not_called()


def test_handle_no_target_writes_telemetry_without_cluster_state(node):
    # A stub exposing no clusters/diag still produces valid no_frontier telemetry.
    node.state = "exploring"
    node.no_target_count = 0
    node.blacklist = set()
    node.telemetry.write = MagicMock()
    node.handle_no_target((0.0, 0.0))
    assert node.no_target_count == 1
    node.telemetry.write.assert_called_once()
    kwargs = node.telemetry.write.call_args.kwargs
    assert kwargs["reason"] == "filtered"
    assert "raw_clusters" not in kwargs  # only present if the algorithm supplies it


def test_marker_hook_payload_published_verbatim(node):
    # When an algorithm supplies render_markers, the node publishes its opaque
    # payload without inspecting it.
    from unittest.mock import MagicMock as MM
    sentinel = object()
    node.algorithm = MockAlgorithm()
    node.algorithm.render_markers = MM(return_value=sentinel)
    node.marker_pub.publish = MM()
    node.publish_markers()
    node.marker_pub.publish.assert_called_once_with(sentinel)


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
    node.no_target_count = 3
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
    node.no_target_count = 0
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
    node.no_target_count = 0
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    published = []
    node.status_pub.publish = lambda m: published.append(json.loads(m.data))
    node.publish_status("exploring")
    assert published[0]["dist_m"] == 3.0


# --- shared ExploreParams wiring (owned by the node) ---

def test_shared_params_default_from_explore_params(node):
    # The node's shared params default to the ExploreParams dataclass values.
    assert node.params.preferred_goal_distance == ExploreParams().preferred_goal_distance
    assert node.params.max_explore_radius == ExploreParams().max_explore_radius


# --- FrontierAlgorithm self-declares its frontier ROS params (F23 T03) ---

def test_frontier_params_defaults_match_dataclass(frontier_node):
    # The frontier ROS params the algorithm declares default to the FrontierParams
    # dataclass values, so yaml/launch overrides layer on a consistent baseline.
    fp = frontier_node.algorithm.frontier_params
    defaults = FrontierParams()
    assert fp.min_frontier_size == defaults.min_frontier_size
    assert fp.min_frontier_dist == defaults.min_frontier_dist
    assert fp.max_frontier_dist == defaults.max_frontier_dist
    assert fp.frontier_buffer_cells == defaults.frontier_buffer_cells
    assert fp.goal_inset_m == defaults.goal_inset_m


def test_frontier_params_declared_as_ros_params(frontier_node):
    # Declared in the node's namespace so they stay yaml/launch settable.
    for name in ("min_frontier_size", "min_frontier_dist", "max_frontier_dist",
                 "frontier_buffer_cells", "goal_inset_m"):
        assert frontier_node.has_parameter(name)


# --- a shared-only plugin runs without the frontier params declared (F23 T03) ---

def test_shared_only_plugin_declares_no_frontier_params(node):
    # MockAlgorithm needs only the shared params; the node must NOT have declared
    # any frontier ROS param on its behalf.
    for name in ("min_frontier_size", "min_frontier_dist", "max_frontier_dist",
                 "frontier_buffer_cells", "goal_inset_m", "prefer_farthest"):
        assert not node.has_parameter(name)


def test_shared_only_plugin_ticks_without_frontier_params(node):
    # A find-and-send tick must run cleanly for a plugin carrying no frontier
    # tuning (frontier_params is None) — no frontier param lookups blow up.
    node.state = "exploring"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.new_goal((1.0, 2.0)))
    node.publish_markers()  # no render_markers hook -> must not raise
    node.find_and_send_goal()
    node.send_nav_goal.assert_called_once()


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
