#!/usr/bin/env python3
# test_explorer_manager_node.py — unit tests for ExplorerManagerNode
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import json
import math
import queue
import threading
import time
from unittest.mock import MagicMock, patch
import pytest
import rclpy
import rclpy.task
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from dome_nav.explore_context import ExploreParams, GoalDecision, GoalOutcome
from dome_nav.frontier_params import FrontierParams
from dome_nav.frontier_algorithm import FrontierAlgorithm
from dome_nav.hello_world_algorithm import HelloWorldAlgorithm


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


# --- ExploreArea session transitions (F35 T07) ---

def test_start_session_from_idle(node):
    node.state = "IDLE"
    node.robot_xy_in_map = MagicMock(return_value=(1.0, 2.0))
    assert node.start_session() is True
    assert node.state == "EXPL"


def test_start_session_from_done(node):
    node.state = "DONE"
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    assert node.start_session() is True
    assert node.state == "EXPL"


def test_start_session_while_exploring_rejected(node):
    node.state = "EXPL"
    node.goal_count = 3
    assert node.start_session() is False
    assert node.state == "EXPL"
    assert node.goal_count == 3  # not reset


def test_start_session_sets_map_name(node):
    node.state = "IDLE"
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.start_session("kitchen")
    assert node.map_name == "kitchen"


def test_stop_exploring_sets_idle(node):
    node.state = "EXPL"
    node.goal_handle = None
    node.stop_exploring("IDLE")
    assert node.state == "IDLE"


def test_explore_goal_rejected_while_exploring(node):
    from rclpy.action import GoalResponse
    node.state = "EXPL"
    assert node.on_explore_goal(object()) == GoalResponse.REJECT


def test_explore_goal_accepted_when_idle(node):
    from rclpy.action import GoalResponse
    node.state = "IDLE"
    node.active_goal_handle = None
    assert node.on_explore_goal(object()) == GoalResponse.ACCEPT


def test_start_session_resets_blacklist_and_counters(node):
    node.state = "IDLE"
    node.blacklist = {(1.0, 2.0), (3.0, 4.0)}
    node.goal_count = 5
    node.goals_reached = 3
    node.goals_failed = 2
    node.no_target_count = 4
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.start_session()
    assert node.blacklist == set()
    assert node.goal_count == 0
    assert node.goals_reached == 0
    assert node.goals_failed == 0
    assert node.no_target_count == 0


# --- ExploreArea feedback + outcome (F35 T07) ---

def test_known_area_none_map_is_zero(node):
    assert node.known_area_m2(None) == 0.0


def test_known_area_counts_known_cells(node):
    grid = OccupancyGrid()
    grid.info.resolution = 0.5
    grid.data = [-1, 0, 100, -1]  # 2 known cells, each 0.25 m^2
    assert node.known_area_m2(grid) == pytest.approx(0.5)


def test_explore_feedback_reports_current_goal(node):
    node.current_goal_xy = (1.5, -2.0)
    node.explored_area_m2 = 3.0
    feedback = node.explore_feedback()
    assert feedback.current_goal.x == pytest.approx(1.5)
    assert feedback.current_goal.y == pytest.approx(-2.0)
    assert feedback.explored_area_m2 == pytest.approx(3.0)


def test_explore_feedback_no_goal_is_origin(node):
    node.current_goal_xy = None
    feedback = node.explore_feedback()
    assert feedback.current_goal.x == 0.0
    assert feedback.current_goal.y == 0.0


def test_explored_done_sets_outcome(node):
    from dome_nav_msgs.action import ExploreArea
    node.state = "EXPL"
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.algorithm = MockAlgorithm(GoalDecision.done())
    node.latest_map = make_map()
    node.find_and_send_goal()
    assert node.state == "DONE"
    assert node.session_outcome == ExploreArea.Result.EXPLORED_DONE


# --- find_and_send_goal via MockAlgorithm ---

def test_find_frontier_no_map_early_return(node):
    node.state = "EXPL"
    node.latest_map = None
    node.send_nav_goal = MagicMock()
    node.find_and_send_goal()
    node.send_nav_goal.assert_not_called()


def test_find_frontier_no_robot_xy_early_return(node):
    node.state = "EXPL"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=None)
    node.send_nav_goal = MagicMock()
    node.find_and_send_goal()
    node.send_nav_goal.assert_not_called()


def test_find_frontier_blocked_increments_count(node):
    node.state = "EXPL"
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
    node.state = "EXPL"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = 0
    node.send_nav_goal = MagicMock()
    node.dump_exhaustion = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.done())
    node.find_and_send_goal()
    assert node.state == "DONE"
    node.send_nav_goal.assert_not_called()


def test_find_frontier_blocked_patience_clears_blacklist_once(node):
    # First patience exhaustion on a block clears the blacklist and retries —
    # it does NOT declare done.
    node.state = "EXPL"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = node.NO_TARGET_PATIENCE - 1
    node.blacklist = {(1.0, 1.0)}
    node.blacklist_cleared_once = False
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.blocked())
    node.find_and_send_goal()
    assert node.state == "EXPL"
    assert node.blacklist == set()
    assert node.blacklist_cleared_once is True
    assert node.no_target_count == 0


def test_find_frontier_blocked_patience_after_clear_sets_done(node):
    # A second patience exhaustion (blacklist already cleared) gives up → done.
    node.state = "EXPL"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = node.NO_TARGET_PATIENCE - 1
    node.blacklist_cleared_once = True
    node.send_nav_goal = MagicMock()
    node.dump_exhaustion = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.blocked())
    node.find_and_send_goal()
    assert node.state == "DONE"
    node.send_nav_goal.assert_not_called()


def test_find_frontier_found_resets_patience_count(node):
    node.state = "EXPL"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.no_target_count = 5
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.new_goal((3.0, 0.0)))
    node.find_and_send_goal()
    assert node.no_target_count == 0
    node.send_nav_goal.assert_called_once()


def test_find_frontier_sends_algorithm_goal(node):
    node.state = "EXPL"
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
    node.state = "EXPL"
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


# --- publish_status JSON shape ---

def test_publish_status_idle_json(node):
    node.goals_reached = 0
    node.goals_failed = 0
    node.robot_xy_in_map = MagicMock(return_value=None)
    published = []
    node.status_pub.publish = lambda m: published.append(json.loads(m.data))
    node.publish_status("IDLE")
    assert published == [{"state": "IDLE", "reached": 0, "failed": 0}]


def test_publish_status_done_carries_counters(node):
    node.goals_reached = 5
    node.goals_failed = 1
    node.robot_xy_in_map = MagicMock(return_value=None)
    published = []
    node.status_pub.publish = lambda m: published.append(json.loads(m.data))
    node.publish_status("DONE")
    assert published[0] == {"state": "DONE", "reached": 5, "failed": 1}


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
    node.publish_status("EXPL")
    d = published[0]
    assert d["state"] == "EXPL"
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
    node.publish_status("EXPL")
    d = published[0]
    assert d["state"] == "EXPL"
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
    node.publish_status("EXPL")
    assert published[0]["dist_m"] == 3.0


# --- shared ExploreParams wiring (owned by the node) ---

def test_shared_params_default_from_explore_params(node):
    # The node's shared params default to the ExploreParams dataclass values.
    defaults = ExploreParams()
    assert node.params.max_explore_radius == defaults.max_explore_radius
    assert node.params.blacklist_radius == defaults.blacklist_radius


def test_blacklist_radius_is_a_ros_param(node):
    # Declared on the node itself (shared param), so it is yaml/launch-settable
    # regardless of which algorithm plugin runs.
    assert node.has_parameter("blacklist_radius")


def test_blacklist_radius_override_reaches_explore_params(ros):
    # Injected at construction (declare-time), mirroring a yaml/launch override.
    from rclpy.node import Node
    from dome_nav.explorer_manager_node import ExplorerManagerNode

    real_declare = Node.declare_parameter

    def declare_with_radius_override(self, name, value=None, *args, **kwargs):
        if name == "blacklist_radius":
            value = 0.7
        return real_declare(self, name, value, *args, **kwargs)

    with patch("tf2_ros.TransformListener"), \
         patch("rclpy.action.ActionClient"), \
         patch("dome_nav.explorer_manager_node.TelemetryWriter",
               return_value=MagicMock()), \
         patch.object(Node, "declare_parameter", declare_with_radius_override):
        n = ExplorerManagerNode(algorithm=MockAlgorithm())
    assert n.params.blacklist_radius == 0.7
    n.destroy_node()


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
                 "frontier_buffer_cells", "goal_inset_m"):
        assert not node.has_parameter(name)


def test_shared_only_plugin_ticks_without_frontier_params(node):
    # A find-and-send tick must run cleanly for a plugin carrying no frontier
    # tuning (frontier_params is None) — no frontier param lookups blow up.
    node.state = "EXPL"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.send_nav_goal = MagicMock()
    node.algorithm = MockAlgorithm(GoalDecision.new_goal((1.0, 2.0)))
    node.publish_markers()  # no render_markers hook -> must not raise
    node.find_and_send_goal()
    node.send_nav_goal.assert_called_once()


# --- goal_within_costmap_bounds bounds check (worldToMap guard) ---

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
    assert node.goal_within_costmap_bounds((5.0, 5.0)) is True


def test_goal_in_costmap_true_for_interior_goal(node):
    node.latest_global_costmap = costmap_2m()
    assert node.goal_within_costmap_bounds((1.0, 1.0)) is True


def test_goal_in_costmap_false_for_goal_past_edge(node):
    # 2m-wide costmap; a goal at x=2.05m maps one cell past the east edge, the
    # exact worldToMap failure that aborted planning with PLAN/NO_VALID_PATH.
    node.latest_global_costmap = costmap_2m()
    assert node.goal_within_costmap_bounds((2.05, 1.0)) is False
    assert node.goal_within_costmap_bounds((1.0, -0.1)) is False


# --- goal_is_lethal guard (F27 T02) ---

def set_cell(cm, xy, cost):
    info = cm.info
    col = int((xy[0] - info.origin.position.x) / info.resolution)
    row = int((xy[1] - info.origin.position.y) / info.resolution)
    cm.data[row * info.width + col] = cost


class SequenceAlgorithm:
    # Yields a fixed sequence of decisions, one per next_goal call, so a test can
    # exercise the reselect loop across multiple candidates.
    frontier_params = None

    def __init__(self, decisions):
        self.decisions = list(decisions)

    def declare_params(self, node):
        pass

    def next_goal(self, ctx):
        return self.decisions.pop(0) if self.decisions else GoalDecision.blocked()


def test_goal_is_lethal_false_when_no_costmap(node):
    # Startup stays permissive before the first costmap arrives.
    node.latest_global_costmap = None
    assert node.goal_is_lethal((1.0, 1.0)) is False


def test_goal_is_lethal_false_for_free_cell(node):
    node.latest_global_costmap = costmap_2m()
    assert node.goal_is_lethal((1.0, 1.0)) is False


def test_goal_is_lethal_true_for_lethal_cell(node):
    cm = costmap_2m()
    set_cell(cm, (1.0, 1.0), 100)  # scaled OccupancyGrid lethal
    node.latest_global_costmap = cm
    assert node.goal_is_lethal((1.0, 1.0)) is True


def test_goal_is_lethal_true_for_inscribed_cell(node):
    # Inscribed (99) = footprint guaranteed in collision -> treated as lethal.
    cm = costmap_2m()
    set_cell(cm, (1.0, 1.0), 99)
    node.latest_global_costmap = cm
    assert node.goal_is_lethal((1.0, 1.0)) is True


def test_goal_is_lethal_false_for_inflation_below_threshold(node):
    cm = costmap_2m()
    set_cell(cm, (1.0, 1.0), 98)  # high inflation but not lethal/inscribed
    node.latest_global_costmap = cm
    assert node.goal_is_lethal((1.0, 1.0)) is False


def test_goal_is_lethal_false_out_of_bounds(node):
    # None cost (out of bounds) is not lethal; bounds are the bounds guard's job.
    node.latest_global_costmap = costmap_2m()
    assert node.goal_is_lethal((2.05, 1.0)) is False


def test_find_and_send_goal_skips_lethal_candidate(node):
    # A lethal (post-nudge) candidate is excluded and next_goal is re-asked; the
    # next free candidate is dispatched instead of aborting on a lethal goal.
    node.state = "EXPL"
    node.latest_map = make_map()
    node.robot_xy_in_map = MagicMock(return_value=(0.0, 0.0))
    node.send_nav_goal = MagicMock()
    cm = costmap_2m()
    set_cell(cm, (1.0, 1.0), 100)  # first candidate lands on lethal
    node.latest_global_costmap = cm
    node.algorithm = SequenceAlgorithm([
        GoalDecision.new_goal((1.0, 1.0)),   # lethal -> skipped
        GoalDecision.new_goal((0.5, 0.5)),   # free  -> sent
    ])
    node.find_and_send_goal()
    node.send_nav_goal.assert_called_once_with((0.5, 0.5))


# --- F22 T03: runtime algorithm selector ---

def test_explore_algorithm_param_declared(node):
    assert node.has_parameter("explore_algorithm")
    assert node.get_parameter("explore_algorithm").value == "frontier"


def test_default_algorithm_is_frontier(frontier_node):
    assert isinstance(frontier_node.algorithm, FrontierAlgorithm)


def test_unknown_explore_algorithm_param_falls_back_to_frontier(ros):
    # An unknown explore_algorithm param value must fall back to the default
    # FrontierAlgorithm and log a warning. The node reads the param during
    # __init__, so the bogus value is injected by patching declare_parameter —
    # a post-construction set_parameter would be too late.
    from rclpy.node import Node
    from dome_nav.explorer_manager_node import ExplorerManagerNode

    real_declare = Node.declare_parameter

    def declare_with_bogus_algorithm(self, name, value=None, *args, **kwargs):
        if name == "explore_algorithm":
            value = "not_a_real_algorithm"
        return real_declare(self, name, value, *args, **kwargs)

    with patch("tf2_ros.TransformListener"), \
         patch("rclpy.action.ActionClient"), \
         patch("dome_nav.explorer_manager_node.TelemetryWriter",
               return_value=MagicMock()), \
         patch.object(Node, "declare_parameter", declare_with_bogus_algorithm), \
         patch.object(ExplorerManagerNode, "get_logger") as mock_get_logger:
        n = ExplorerManagerNode()
    assert isinstance(n.algorithm, FrontierAlgorithm)
    mock_get_logger.return_value.warning.assert_called_once()
    n.destroy_node()


# --- failure counter covers watchdog paths (telemetry honesty regression) ---

def test_timeout_increments_goals_failed(node):
    node.has_active_goal = True
    node.goal_start_time = 0.0
    node.goal_handle = MagicMock()
    node.current_goal_xy = (4.7, 5.0)
    node.goals_failed = 0
    node.check_goal_timeout()
    assert node.goals_failed == 1


def test_stuck_increments_goals_failed(node):
    node.has_active_goal = True
    node.goal_start_time = 0.0
    node.goal_handle = MagicMock()
    node.current_goal_xy = (5.0, 0.0)
    node.robot_xy_in_map = lambda: (0.0, 0.0)
    node.best_dist_to_goal = 5.0
    node.last_progress_xy = (0.0, 0.0)
    node.last_progress_time = 0.0  # ancient → stuck window expired
    node.goals_failed = 0
    node.check_stuck()
    assert node.goals_failed == 1


# --- wedge detector: consecutive same-pose stucks stop the session ---

def make_stuck(node, goal_xy, robot_xy):
    node.has_active_goal = True
    node.goal_start_time = 0.0
    node.goal_handle = MagicMock()
    node.current_goal_xy = goal_xy
    node.robot_xy_in_map = lambda: robot_xy
    node.best_dist_to_goal = math.dist(robot_xy, goal_xy)  # no progress made
    node.last_progress_xy = robot_xy
    node.last_progress_time = 0.0
    node.check_stuck()


def test_wedge_same_pose_stucks_stop_session(node):
    node.state = "EXPL"
    for i in range(node.WEDGED_STUCK_LIMIT):
        make_stuck(node, (5.0 + i, 0.0), (1.0, 1.0))
    assert node.state == "IDLE"
    assert node.stuck_streak == node.WEDGED_STUCK_LIMIT


def test_wedge_streak_resets_when_pose_changes(node):
    node.state = "EXPL"
    make_stuck(node, (5.0, 0.0), (1.0, 1.0))
    make_stuck(node, (6.0, 0.0), (2.0, 2.0))  # moved — streak restarts
    make_stuck(node, (7.0, 0.0), (2.0, 2.0))
    assert node.state == "EXPL"
    assert node.stuck_streak == 2


def test_wedge_streak_resets_on_reached_goal(node):
    node.state = "EXPL"
    make_stuck(node, (5.0, 0.0), (1.0, 1.0))
    make_stuck(node, (6.0, 0.0), (1.0, 1.0))
    assert node.stuck_streak == 2
    node.has_active_goal = True
    node.current_goal_xy = (2.0, 2.0)
    node.goal_start_time = time.monotonic()
    node.on_goal_result(
        goal_result_future(GoalStatus.STATUS_SUCCEEDED), xy=(2.0, 2.0)
    )
    assert node.stuck_streak == 0
    assert node.state == "EXPL"


# --- goal rejection clears full active-goal state (stale status regression) ---

def test_rejected_goal_clears_all_active_state(node):
    node.has_active_goal = True
    node.current_goal_xy = (3.0, 4.0)
    node.goal_start_time = time.monotonic()
    node.latest_global_costmap = None
    node.robot_xy_in_map = lambda: None
    future = MagicMock()
    future.result.return_value.accepted = False
    node.on_goal_accepted(future, xy=(3.0, 4.0))
    assert node.has_active_goal is False
    assert node.current_goal_xy is None
    assert node.goal_start_time is None
    assert (3.0, 4.0) in node.blacklist


# --- on_goal_result stale-callback guard (race regression) ---

def test_on_goal_result_ignores_cleared_goal(node):
    # A goal canceled by the watchdog/stop clears goal_start_time to None; the late
    # result callback must not crash (was: float - None TypeError) or mutate state.
    node.has_active_goal = False
    node.current_goal_xy = None
    node.goal_start_time = None
    node.goals_failed = 0
    node.blacklist = set()
    node.on_goal_result(MagicMock(), xy=(1.0, 2.0))  # must not raise
    assert node.goals_failed == 0
    assert node.blacklist == set()


def test_on_goal_result_ignores_superseded_goal(node):
    # A newer goal is active; the previous goal's late callback must not blacklist
    # its own xy or touch the current goal's state.
    node.has_active_goal = True
    node.current_goal_xy = (9.0, 9.0)
    node.goal_start_time = time.monotonic()
    node.goals_failed = 0
    node.blacklist = set()
    node.on_goal_result(MagicMock(), xy=(1.0, 2.0))
    assert node.goals_failed == 0
    assert (1.0, 2.0) not in node.blacklist


def goal_result_future(status: int) -> MagicMock:
    future = MagicMock()
    future.result.return_value.status = status
    return future


def test_on_goal_result_reached_not_blacklisted(node):
    # Regression: successes were blacklisted too, and the blacklist_radius circle
    # around every reached goal killed live frontier cells — over-accumulation
    # ended sessions prematurely. Only failures blacklist.
    node.has_active_goal = True
    node.current_goal_xy = (1.0, 2.0)
    node.goal_start_time = time.monotonic()
    node.blacklist = set()
    node.on_goal_result(
        goal_result_future(GoalStatus.STATUS_SUCCEEDED), xy=(1.0, 2.0)
    )
    assert node.goals_reached == 1
    assert node.blacklist == set()


def test_on_goal_result_failure_still_blacklisted(node):
    node.has_active_goal = True
    node.current_goal_xy = (1.0, 2.0)
    node.goal_start_time = time.monotonic()
    node.blacklist = set()
    node.on_goal_result(
        goal_result_future(GoalStatus.STATUS_CANCELED), xy=(1.0, 2.0)
    )
    assert node.goals_failed == 1
    assert (1.0, 2.0) in node.blacklist


# --- I01 (dome_mission issue tracker): explore_tick's on-demand grid fetch
# races a concurrently spinning executor. `fetch_grid` (via rclpy's
# `wait_for_message`) creates a subscription, blocks on a private WaitSet
# outside the executor, then destroys the subscription — mutating the node's
# entity list from whatever thread calls it. `explore_tick` (a real 1Hz timer
# callback) calls it whenever there's no active goal, from a worker thread
# under the MultiThreadedExecutor + ReentrantCallbackGroup this node runs
# under (see main()), concurrently with the main thread's own wait-set-rebuild
# loop. When a destroy lands mid-rebuild, rclpy raises
# `InvalidHandle: cannot use Destroyable because destruction was requested`,
# which is exactly what crashed explorer_manager_node live (dome_mission
# 05-issues/open/I01-explore-manager-crash-on-completion.md).
#
# This test never calls fetch_grid (or explore_tick) directly — it only ever
# calls the real public start_session(), the same entry point
# execute_explore uses, then lets the node's own real timer drive
# explore_tick -> find_and_send_goal -> fetch_grid exactly as production
# does. The one liberty taken for test speed: EXPLORE_HZ is patched higher
# before construction (the timer period is fixed at __init__ time), so many
# real ticks happen per second instead of one. With MockAlgorithm's default
# "always blocked" decision, no nav goal is ever sent, so has_active_goal
# stays False and every tick re-enters the fetch_grid branch — the session
# naturally ends after NO_TARGET_PATIENCE-driven ticks, so the test restarts
# it via start_session() in a loop to keep generating fresh fetch_grid calls
# for the whole test window.
#
# Timing-dependent by nature: it fails on the buggy fetch_grid most runs
# within the time budget below, but a race is never 100% guaranteed on any
# given run. Not part of the plain fast suite; marked manual.
#
# Catching it: the InvalidHandle exception doesn't reliably propagate out of
# executor.spin() itself — the MultiThreadedExecutor dispatches ready-waitable
# handling through its own rclpy.task.Future/Task machinery, and a raise
# inside that gets stashed via Future.set_exception() rather than raised
# synchronously. If nothing calls .result()/.exception() on that Future, it
# is silently dropped except for a stderr print from Future.__del__ at GC
# time ("The following exception was never retrieved: ...") — easy to miss
# and not something a bare try/except around spin() catches. So this test
# patches rclpy.task.Future.set_exception for its duration to capture every
# exception the executor's internal task machinery records, in addition to
# the try/except around spin() itself (belt and suspenders: some failures
# may still raise synchronously on the spin thread).


@pytest.mark.manual
def test_explore_tick_survives_concurrent_executor_spin(ros):
    from dome_nav.explorer_manager_node import ExplorerManagerNode

    with patch("tf2_ros.TransformListener"), \
         patch("rclpy.action.ActionClient"), \
         patch("dome_nav.explorer_manager_node.TelemetryWriter",
               return_value=MagicMock()), \
         patch.object(ExplorerManagerNode, "EXPLORE_HZ", 200.0):
        node = ExplorerManagerNode(algorithm=MockAlgorithm())

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    errors: queue.Queue = queue.Queue()

    def spin():
        try:
            executor.spin()
        except Exception as exc:
            errors.put(exc)

    original_set_exception = rclpy.task.Future.set_exception

    def capturing_set_exception(self, exception):
        errors.put(exception)
        original_set_exception(self, exception)

    spin_thread = threading.Thread(target=spin, daemon=True)
    with patch.object(rclpy.task.Future, "set_exception", capturing_set_exception):
        spin_thread.start()
        try:
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and errors.empty():
                if node.state in ("IDLE", "DONE"):
                    node.start_session()
                time.sleep(0.02)
        finally:
            executor.shutdown()
            spin_thread.join(timeout=5.0)
            node.destroy_node()

    assert errors.empty(), f"executor thread crashed: {errors.get()!r}"
