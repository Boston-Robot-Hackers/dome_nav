#!/usr/bin/env python3
# test_frontier_algorithm.py — unit tests for FrontierAlgorithm (pure Python, no ROS2)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import math
from dome_nav.explore_context import (
    ExplorationContext, ExploreParams, GoalOutcome,
)
from dome_nav.frontier_params import FrontierParams
from dome_nav.frontier_algorithm import FrontierAlgorithm
from dome_nav.frontier_explorer import MapInfo


# Default frontier tuning for the tiny test maps: accept single-cell clusters,
# no min-distance floor, one buffer ring. Frontier tuning now lives in
# FrontierParams (F23 T03), constructed into the algorithm, not in ExploreParams.
def make_frontier_params(**overrides) -> FrontierParams:
    base = dict(min_frontier_size=1, min_frontier_dist=0.0, frontier_buffer_cells=1)
    base.update(overrides)
    return FrontierParams(**base)


def make_algo(frontier: FrontierParams | None = None) -> FrontierAlgorithm:
    return FrontierAlgorithm(frontier_params=frontier or make_frontier_params())


def make_info(width: int, height: int, resolution: float = 1.0) -> MapInfo:
    return MapInfo(
        width=width, height=height, resolution=resolution,
        origin_x=0.0, origin_y=0.0,
    )


def flat_map(width: int, height: int, value: int) -> list[int]:
    return [value] * (width * height)


def make_ctx(
    map_data: list[int],
    map_info: MapInfo,
    robot_xy: tuple[float, float] = (0.0, 0.0),
    blacklist: set[tuple[float, float]] | None = None,
    start_xy: tuple[float, float] | None = None,
    shared: ExploreParams | None = None,
) -> ExplorationContext:
    return ExplorationContext(
        map_data=map_data,
        map_info=map_info,
        robot_xy=robot_xy,
        blacklist=blacklist or set(),
        start_xy=start_xy,
        params=shared or ExploreParams(),
    )


# --- no raw clusters → EXPLORED_DONE (algorithm owns the done-condition) ---

def test_next_goal_no_frontiers_all_free():
    algo = make_algo()
    info = make_info(3, 3)
    ctx = make_ctx(flat_map(3, 3, 0), info)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.EXPLORED_DONE
    assert decision.xy is None


def test_next_goal_no_frontiers_all_unknown():
    algo = make_algo()
    info = make_info(3, 3)
    ctx = make_ctx(flat_map(3, 3, -1), info)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.EXPLORED_DONE
    assert decision.xy is None


# --- next_goal returns NEW_GOAL carrying an (x, y) on a map with frontier cells ---

def test_next_goal_returns_xy_on_frontier_map():
    algo = make_algo()
    info = make_info(5, 1)
    # [free, free, free, unknown, unknown] → cells 0-2 free, 3-4 unknown
    # Cell 2 is free with unknown neighbor at cell 3 → frontier
    data = [0, 0, 0, -1, -1]
    ctx = make_ctx(data, info)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    assert isinstance(decision.xy, tuple)
    assert len(decision.xy) == 2


# --- preferred_goal_distance plumbed through from the shared ExploreParams ---

def test_next_goal_large_preferred_dist_picks_far_cluster():
    algo = make_algo()
    info = make_info(10, 1)
    # free cells 2, 4, 8 each border an unknown neighbor -> 3 separate frontier
    # clusters at world x=2.5, 4.5, 8.5. Robot at x=0.
    data = [0, 0, 0, -1, 0, 0, 0, 0, 0, -1]
    shared = ExploreParams(preferred_goal_distance=1000.0)
    ctx = make_ctx(data, info, shared=shared)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    assert decision.xy[0] > 7.0


def test_next_goal_zero_preferred_dist_picks_near_cluster():
    algo = make_algo()
    info = make_info(10, 1)
    data = [0, 0, 0, -1, 0, 0, 0, 0, 0, -1]
    shared = ExploreParams(preferred_goal_distance=0.0)
    ctx = make_ctx(data, info, shared=shared)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    assert decision.xy[0] < 3.0


# --- FrontierAlgorithm reads its tuning from FrontierParams (F23 T03) ---

def test_min_frontier_size_from_frontier_params_filters_small_cluster():
    # A 3-cell frontier is emitted with min_frontier_size=1 but suppressed (blocked)
    # when FrontierParams raises the floor to 5 — proving the algorithm reads
    # min_frontier_size from its FrontierParams, not from ExploreParams.
    info = make_info(6, 1)
    data = [0, 0, 0, 0, -1, -1]  # cells 1,2 form the buffer-ring frontier cluster
    ctx = make_ctx(data, info)
    permissive = make_algo(make_frontier_params(min_frontier_size=1))
    assert permissive.next_goal(ctx).outcome is GoalOutcome.NEW_GOAL
    strict = make_algo(make_frontier_params(min_frontier_size=5))
    assert strict.next_goal(ctx).outcome is GoalOutcome.NO_TARGETS_BLOCKED


def test_min_frontier_dist_from_frontier_params_filters_near_cell():
    # The sole frontier cell sits ~2.5m from the robot; a min_frontier_dist floor
    # above that distance filters it out -> blocked, showing min_frontier_dist is
    # read from FrontierParams.
    info = make_info(5, 1)
    data = [0, 0, 0, -1, -1]
    ctx = make_ctx(data, info, robot_xy=(0.0, 0.0))
    near_ok = make_algo(make_frontier_params(min_frontier_dist=0.0))
    assert near_ok.next_goal(ctx).outcome is GoalOutcome.NEW_GOAL
    too_close = make_algo(make_frontier_params(min_frontier_dist=100.0))
    assert too_close.next_goal(ctx).outcome is GoalOutcome.NO_TARGETS_BLOCKED


def test_frontier_buffer_cells_from_frontier_params_changes_frontier():
    # buffer_cells=0 keeps the frontier at the boundary cell touching unknown;
    # buffer_cells=2 pushes it two known cells inward, landing on a different world
    # x. Reading the value from FrontierParams changes which cell is chosen.
    info = make_info(6, 1)
    data = [0, 0, 0, 0, -1, -1]
    ctx = make_ctx(data, info, robot_xy=(0.0, 0.0))
    at_boundary = make_algo(
        make_frontier_params(frontier_buffer_cells=0, goal_inset_m=0.0)
    ).next_goal(ctx)
    inset = make_algo(
        make_frontier_params(frontier_buffer_cells=2, goal_inset_m=0.0)
    ).next_goal(ctx)
    assert at_boundary.outcome is GoalOutcome.NEW_GOAL
    assert inset.outcome is GoalOutcome.NEW_GOAL
    assert at_boundary.xy[0] != inset.xy[0]


def test_prefer_farthest_from_frontier_params_picks_far_cluster():
    # Deprecated prefer_farthest lives in FrontierParams; True makes selection
    # farthest-first regardless of the shared preferred_goal_distance.
    info = make_info(10, 1)
    data = [0, 0, 0, -1, 0, 0, 0, 0, 0, -1]
    ctx = make_ctx(data, info, shared=ExploreParams(preferred_goal_distance=0.0))
    algo = make_algo(make_frontier_params(prefer_farthest=True))
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    assert decision.xy[0] > 7.0


# --- latest_clusters populated after each call ---

def test_latest_clusters_populated_after_call():
    algo = make_algo()
    assert algo.latest_clusters == []
    info = make_info(5, 1)
    data = [0, 0, 0, -1, -1]
    ctx = make_ctx(data, info)
    algo.next_goal(ctx)
    assert len(algo.latest_clusters) >= 1


def test_latest_clusters_empty_on_explored_map():
    algo = make_algo()
    info = make_info(3, 3)
    ctx = make_ctx(flat_map(3, 3, 0), info)
    algo.next_goal(ctx)
    assert algo.latest_clusters == []


# --- latest_diag set when None, cleared otherwise ---

def test_latest_diag_set_when_no_frontier():
    algo = make_algo()
    info = make_info(3, 3)
    ctx = make_ctx(flat_map(3, 3, 0), info)
    algo.next_goal(ctx)
    assert algo.latest_diag is not None
    assert isinstance(algo.latest_diag, dict)


def test_latest_diag_cleared_when_frontier_found():
    algo = make_algo()
    info = make_info(5, 1)
    data = [0, 0, 0, -1, -1]
    ctx = make_ctx(data, info)
    algo.next_goal(ctx)
    assert algo.latest_diag is None


def test_latest_diag_transitions():
    algo = make_algo()
    info = make_info(5, 1)
    ctx_empty = make_ctx(flat_map(5, 1, 0), info)
    algo.next_goal(ctx_empty)
    assert algo.latest_diag is not None
    ctx_frontier = make_ctx([0, 0, 0, -1, -1], info)
    algo.next_goal(ctx_frontier)
    assert algo.latest_diag is None


# --- clusters present but all blacklisted → NO_TARGETS_BLOCKED, not done ---

def test_blacklist_causes_blocked_when_only_frontier_filtered():
    info = make_info(6, 1)
    # 6x1: cells 0-3 free, 4-5 unknown. Cell 3 touches unknown directly and
    # is excluded under the buffer-cell rule; cell 2 (x=2.5) is the buffer
    # cell and the sole frontier.
    data = [0, 0, 0, 0, -1, -1]
    blacklist = {(2.5, 0.5)}
    algo = make_algo()
    ctx = make_ctx(
        data, info, blacklist=blacklist,
        shared=ExploreParams(blacklist_radius=1.0),
    )
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NO_TARGETS_BLOCKED
    assert decision.xy is None
    # Raw clusters do exist — this is a block, not exploration completion.
    assert len(algo.latest_clusters) >= 1


# --- goal_inset_m nudge is applied ---

def test_nudge_applied_goal_closer_than_raw_cell():
    algo = make_algo(make_frontier_params(goal_inset_m=0.3))
    # 10x1 map: cells 0-4 free, 5-9 unknown → frontier near cell 4 (x=4.5)
    info = make_info(10, 1)
    data = [0] * 5 + [-1] * 5
    ctx = make_ctx(data, info, robot_xy=(0.0, 0.0))
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    result = decision.xy
    raw_dist = 4.5  # approximate distance of nearest frontier cell
    result_dist = math.sqrt(result[0] ** 2 + result[1] ** 2)
    assert result_dist < raw_dist


def test_nudge_amount_correct():
    inset = 0.3
    algo = make_algo(make_frontier_params(goal_inset_m=inset))
    # 10x1 map, cells 0-4 free, 5-9 unknown. Cell 4 touches unknown directly and
    # is excluded under the buffer-cell rule; cell 3 (x=3.5, y=0.5) is the buffer
    # cell and the sole frontier. Robot at origin.
    info = make_info(10, 1)
    data = [0] * 5 + [-1] * 5
    ctx = make_ctx(data, info, robot_xy=(0.0, 0.0))
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    result = decision.xy
    raw_xy = (3.5, 0.5)
    raw_dist = math.sqrt(raw_xy[0] ** 2 + raw_xy[1] ** 2)
    result_dist = math.sqrt(result[0] ** 2 + result[1] ** 2)
    assert abs((raw_dist - result_dist) - inset) < 1e-6
