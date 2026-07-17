#!/usr/bin/env python3
# test_explore_diagnostics.py — pure tests for extracted diagnostic formatters
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from nav_msgs.msg import OccupancyGrid
from dome_nav.frontier_params import FrontierTuning
from dome_nav.explore_diagnostics import (
    cluster_centroid,
    costmap_cell_cost,
    costmap_radius_costs,
    exhaustion_reason,
    format_cluster_summary,
    format_failure_diagnostics,
    format_frontier_exhaustion,
)
from dome_nav.frontier_explorer import MapInfo


# The diagnostics formatters take the combined FrontierTuning (F23 T03). Build one
# with geometry-friendly filter defaults; override the frontier fields per test.
def tuning(
    min_frontier_size: int = 1,
    min_frontier_dist: float = 0.0,
    max_frontier_dist: float = 0.0,
) -> FrontierTuning:
    return FrontierTuning(
        min_frontier_size=min_frontier_size,
        blacklist_radius=0.5,
        min_frontier_dist=min_frontier_dist,
        max_frontier_dist=max_frontier_dist,
        goal_inset_m=0.3,
        max_explore_radius=0.0,
        preferred_goal_distance=1.0,
        frontier_buffer_cells=2,
        prefer_farthest=False,
        use_novelty_scoring=False,
        novelty_top_n=5,
    )


def make_info(width=10, height=10, res=1.0):
    return MapInfo(width=width, height=height, resolution=res,
                   origin_x=0.0, origin_y=0.0)


def make_costmap(width=5, height=5, res=1.0, fill=0):
    cm = OccupancyGrid()
    cm.info.width = width
    cm.info.height = height
    cm.info.resolution = res
    cm.info.origin.position.x = 0.0
    cm.info.origin.position.y = 0.0
    cm.data = [fill] * (width * height)
    return cm


def test_cluster_centroid_averages_cells():
    info = make_info()
    # cells 0 and 2 → world x=0.5 and 2.5 → centroid x=1.5, y=0.5
    cx, cy = cluster_centroid([0, 2], info)
    assert abs(cx - 1.5) < 1e-6
    assert abs(cy - 0.5) < 1e-6


def test_costmap_cell_cost_none_when_no_map():
    assert costmap_cell_cost(None, (0.0, 0.0)) is None


def test_costmap_cell_cost_none_when_out_of_bounds():
    cm = make_costmap()
    assert costmap_cell_cost(cm, (99.0, 99.0)) is None


def test_costmap_cell_cost_reads_value():
    cm = make_costmap(fill=100)
    assert costmap_cell_cost(cm, (2.5, 2.5)) == 100


def test_costmap_radius_costs_na_when_no_map():
    assert costmap_radius_costs(None, (0.0, 0.0)) == "n/a"


def test_costmap_radius_negative_renders_unknown():
    # OccupancyGrid data is int8; -1 is the reachable "unknown" value.
    cm = make_costmap(fill=-1)
    assert "???" in costmap_radius_costs(cm, (2.5, 2.5), radius_cells=1)


def test_exhaustion_reason_too_small():
    params = tuning(min_frontier_size=10, min_frontier_dist=0.0)
    assert "too_small" in exhaustion_reason(3, 5.0, params)


def test_exhaustion_reason_all_blacklisted():
    params = tuning(min_frontier_size=1)
    assert "all_blacklisted" in exhaustion_reason(5, float("inf"), params)


def test_exhaustion_reason_ok():
    params = tuning(min_frontier_size=1, min_frontier_dist=0.0, max_frontier_dist=0.0)
    assert exhaustion_reason(5, 2.0, params) == "OK"


def test_format_frontier_exhaustion_returns_string():
    info = make_info()
    params = tuning(min_frontier_size=1)
    out = format_frontier_exhaustion([[0, 1, 2]], info, (0.0, 0.0), params, set(), 14)
    assert "FRONTIER EXHAUSTION" in out
    assert "patience=14" in out


def test_format_failure_diagnostics_returns_string():
    out = format_failure_diagnostics(
        (1.0, 1.0), (0.0, 0.0), "aborted", 12.0, 3,
        None, None, set(), nav2_error_code=205,
    )
    assert "NAV FAILURE" in out
    assert "PLAN/START_OCCUPIED" in out


def test_format_failure_diagnostics_appends_algorithm_report():
    out = format_failure_diagnostics(
        (1.0, 1.0), (0.0, 0.0), "aborted", 12.0, 3,
        None, None, set(), algorithm_report="  frontiers: 2 clusters available",
    )
    assert "frontiers: 2 clusters available" in out


def test_format_cluster_summary_lists_clusters():
    info = make_info()
    out = format_cluster_summary([[0, 1, 2]], info)
    assert "frontiers: 1 clusters available" in out
    assert "size=3" in out
