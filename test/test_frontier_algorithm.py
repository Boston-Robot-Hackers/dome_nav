#!/usr/bin/env python3
# test_frontier_algorithm.py — unit tests for FrontierAlgorithm (pure Python, no ROS2)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import math
import pytest
from dome_nav.explore_context import (
    ExplorationContext, ExploreParams, GoalOutcome,
)
from dome_nav.frontier_algorithm import FrontierAlgorithm
from dome_nav.frontier_explorer import MapInfo


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
    params: ExploreParams | None = None,
) -> ExplorationContext:
    return ExplorationContext(
        map_data=map_data,
        map_info=map_info,
        robot_xy=robot_xy,
        blacklist=blacklist or set(),
        start_xy=start_xy,
        params=params or ExploreParams(
            min_frontier_size=1, min_frontier_dist=0.0, frontier_buffer_cells=1
        ),
    )


# --- no raw clusters → EXPLORED_DONE (algorithm owns the done-condition) ---

def test_next_goal_no_frontiers_all_free():
    algo = FrontierAlgorithm()
    info = make_info(3, 3)
    ctx = make_ctx(flat_map(3, 3, 0), info)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.EXPLORED_DONE
    assert decision.xy is None


def test_next_goal_no_frontiers_all_unknown():
    algo = FrontierAlgorithm()
    info = make_info(3, 3)
    ctx = make_ctx(flat_map(3, 3, -1), info)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.EXPLORED_DONE
    assert decision.xy is None


# --- next_goal returns NEW_GOAL carrying an (x, y) on a map with frontier cells ---

def test_next_goal_returns_xy_on_frontier_map():
    algo = FrontierAlgorithm()
    info = make_info(5, 1)
    # [free, free, free, unknown, unknown] → cells 0-2 free, 3-4 unknown
    # Cell 2 is free with unknown neighbor at cell 3 → frontier
    data = [0, 0, 0, -1, -1]
    ctx = make_ctx(data, info)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    assert isinstance(decision.xy, tuple)
    assert len(decision.xy) == 2


# --- preferred_goal_distance plumbed through from ExploreParams ---

def test_next_goal_large_preferred_dist_picks_far_cluster():
    algo = FrontierAlgorithm()
    info = make_info(10, 1)
    # free cells 2, 4, 8 each border an unknown neighbor -> 3 separate frontier
    # clusters at world x=2.5, 4.5, 8.5. Robot at x=0.
    data = [0, 0, 0, -1, 0, 0, 0, 0, 0, -1]
    params = ExploreParams(
        min_frontier_size=1, min_frontier_dist=0.0, preferred_goal_distance=1000.0,
        frontier_buffer_cells=1,
    )
    ctx = make_ctx(data, info, params=params)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    assert decision.xy[0] > 7.0


def test_next_goal_zero_preferred_dist_picks_near_cluster():
    algo = FrontierAlgorithm()
    info = make_info(10, 1)
    data = [0, 0, 0, -1, 0, 0, 0, 0, 0, -1]
    params = ExploreParams(
        min_frontier_size=1, min_frontier_dist=0.0, preferred_goal_distance=0.0,
        frontier_buffer_cells=1,
    )
    ctx = make_ctx(data, info, params=params)
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    assert decision.xy[0] < 3.0


# --- latest_clusters populated after each call ---

def test_latest_clusters_populated_after_call():
    algo = FrontierAlgorithm()
    assert algo.latest_clusters == []
    info = make_info(5, 1)
    data = [0, 0, 0, -1, -1]
    ctx = make_ctx(data, info)
    algo.next_goal(ctx)
    assert len(algo.latest_clusters) >= 1


def test_latest_clusters_empty_on_explored_map():
    algo = FrontierAlgorithm()
    info = make_info(3, 3)
    ctx = make_ctx(flat_map(3, 3, 0), info)
    algo.next_goal(ctx)
    assert algo.latest_clusters == []


# --- latest_diag set when None, cleared otherwise ---

def test_latest_diag_set_when_no_frontier():
    algo = FrontierAlgorithm()
    info = make_info(3, 3)
    ctx = make_ctx(flat_map(3, 3, 0), info)
    algo.next_goal(ctx)
    assert algo.latest_diag is not None
    assert isinstance(algo.latest_diag, dict)


def test_latest_diag_cleared_when_frontier_found():
    algo = FrontierAlgorithm()
    info = make_info(5, 1)
    data = [0, 0, 0, -1, -1]
    ctx = make_ctx(data, info)
    # First call with frontiers → diag cleared
    algo.next_goal(ctx)
    assert algo.latest_diag is None


def test_latest_diag_transitions():
    algo = FrontierAlgorithm()
    info = make_info(5, 1)
    # Call with no frontiers → diag set
    ctx_empty = make_ctx(flat_map(5, 1, 0), info)
    algo.next_goal(ctx_empty)
    assert algo.latest_diag is not None
    # Call with frontiers → diag cleared
    ctx_frontier = make_ctx([0, 0, 0, -1, -1], info)
    algo.next_goal(ctx_frontier)
    assert algo.latest_diag is None


# --- clusters present but all blacklisted → NO_TARGETS_BLOCKED, not done ---

def test_blacklist_causes_blocked_when_only_frontier_filtered():
    algo = FrontierAlgorithm()
    info = make_info(6, 1)
    # 6x1: cells 0-3 free, 4-5 unknown. Cell 3 touches unknown directly and
    # is excluded under the buffer-cell rule; cell 2 (x=2.5) is the buffer
    # cell and the sole frontier.
    data = [0, 0, 0, 0, -1, -1]
    blacklist = {(2.5, 0.5)}
    ctx = make_ctx(data, info, blacklist=blacklist,
                   params=ExploreParams(min_frontier_size=1, min_frontier_dist=0.0,
                                        blacklist_radius=1.0,
                                        frontier_buffer_cells=1))
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NO_TARGETS_BLOCKED
    assert decision.xy is None
    # Raw clusters do exist — this is a block, not exploration completion.
    assert len(algo.latest_clusters) >= 1


# --- goal_inset_m nudge is applied ---

def test_nudge_applied_goal_closer_than_raw_cell():
    algo = FrontierAlgorithm()
    # 10x1 map: cells 0-4 free, 5-9 unknown → frontier at cell 4 (x=4.5)
    info = make_info(10, 1)
    data = [0] * 5 + [-1] * 5
    robot_xy = (0.0, 0.0)
    inset = 0.3
    ctx = make_ctx(data, info, robot_xy=robot_xy,
                   params=ExploreParams(min_frontier_size=1, min_frontier_dist=0.0,
                                        goal_inset_m=inset,
                                        frontier_buffer_cells=1))
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    result = decision.xy
    # The raw frontier cell is at x=4.5 (or nearby). The nudged result
    # should be closer to the robot than the raw cell.
    raw_dist = 4.5  # approximate distance of nearest frontier cell
    result_dist = math.sqrt(result[0] ** 2 + result[1] ** 2)
    assert result_dist < raw_dist


def test_nudge_amount_correct():
    algo = FrontierAlgorithm()
    # 10x1 map, cells 0-4 free, 5-9 unknown. Cell 4 touches unknown directly
    # and is excluded under the buffer-cell rule; cell 3 (x=3.5, y=0.5) is
    # the buffer cell and the sole frontier. Robot at origin.
    info = make_info(10, 1)
    data = [0] * 5 + [-1] * 5
    robot_xy = (0.0, 0.0)
    inset = 0.3
    ctx = make_ctx(data, info, robot_xy=robot_xy,
                   params=ExploreParams(min_frontier_size=1, min_frontier_dist=0.0,
                                        goal_inset_m=inset,
                                        frontier_buffer_cells=1))
    decision = algo.next_goal(ctx)
    assert decision.outcome is GoalOutcome.NEW_GOAL
    result = decision.xy
    # After nudge toward (0,0) by 0.3m, the distance should be reduced by
    # exactly 0.3m.
    raw_xy = (3.5, 0.5)
    raw_dist = math.sqrt(raw_xy[0] ** 2 + raw_xy[1] ** 2)
    result_dist = math.sqrt(result[0] ** 2 + result[1] ** 2)
    assert abs((raw_dist - result_dist) - inset) < 1e-6
