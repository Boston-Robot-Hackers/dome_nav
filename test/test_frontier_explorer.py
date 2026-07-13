#!/usr/bin/env python3
# test_frontier_explorer.py — unit tests for FrontierExplorer pure logic
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import math
import pytest
from dome_nav.explore_context import ExploreParams
from dome_nav.frontier_explorer import (
    MapInfo,
    frontier_diag,
    cell_to_world,
    find_frontier_clusters,
    nudge_toward_robot,
    pick_best_frontier,
)


def make_info(width: int, height: int, resolution: float = 1.0) -> MapInfo:
    return MapInfo(width=width, height=height, resolution=resolution, origin_x=0.0,
                   origin_y=0.0)


# pick_best_frontier now takes an ExploreParams. These pure geometry tests want
# no-op distance filters by default (min_frontier_dist=0.0), unlike the operational
# ExploreParams default of 1.3 m — so build filters here with geometry-friendly
# defaults and override per test.
def filters(min_frontier_size: int = 1, min_frontier_dist: float = 0.0, **kw):
    return ExploreParams(
        min_frontier_size=min_frontier_size, min_frontier_dist=min_frontier_dist, **kw
    )


def flat_map(width: int, height: int, value: int) -> list[int]:
    return [value] * (width * height)


# --- find_frontier_clusters ---

def test_no_frontiers_all_unknown():
    info = make_info(3, 3)
    data = flat_map(3, 3, -1)
    assert find_frontier_clusters(data, info) == []


def test_no_frontiers_all_free():
    info = make_info(3, 3)
    data = flat_map(3, 3, 0)
    assert find_frontier_clusters(data, info) == []


def test_no_frontiers_all_occupied():
    info = make_info(3, 3)
    data = flat_map(3, 3, 100)
    assert find_frontier_clusters(data, info) == []


def test_single_frontier_cell():
    # 1x5: [-1, 0, 0, 0, -1]. Cells 1 and 3 touch unknown directly and are
    # excluded as frontiers under the buffer-cell rule; cell 2 (the buffer
    # cell, adjacent to both but not itself touching unknown) is the sole
    # frontier candidate.
    info = make_info(5, 1)
    data = [-1, 0, 0, 0, -1]
    clusters = find_frontier_clusters(data, info, buffer_cells=1)
    assert len(clusters) == 1
    assert clusters[0] == [2]


def test_buffer_cells_zero_returns_boundary_cells():
    # buffer_cells=0 → frontier is the boundary itself (cells touching unknown).
    # 1x5: [-1, 0, 0, 0, -1] — cells 1 and 3 touch unknown, cell 2 does not.
    # Cells 1 and 3 are two apart (not 8-adjacent) so they form two clusters.
    info = make_info(5, 1)
    data = [-1, 0, 0, 0, -1]
    clusters = find_frontier_clusters(data, info, buffer_cells=0)
    assert sorted(c[0] for c in clusters) == [1, 3]


def test_two_separate_clusters():
    # Two independent 5-cell "single frontier cell" patterns (see above)
    # concatenated with a 1-cell unknown gap between them, giving two
    # buffer-cell frontiers (local index 2 and 7) far enough apart to stay
    # in separate clusters.
    info = make_info(10, 1)
    data = [-1, 0, 0, 0, -1, -1, 0, 0, 0, -1]
    clusters = find_frontier_clusters(data, info, buffer_cells=1)
    assert len(clusters) == 2
    total_cells = sum(len(c) for c in clusters)
    assert total_cells == 2


def test_occupied_cell_not_frontier():
    # 1x6: [occupied, unknown, free, free, free, unknown]. Cell 2 touches
    # unknown directly (excluded under the buffer rule); cell 3 is the
    # buffer cell and the sole frontier. The occupied cell must never
    # appear in any cluster.
    info = make_info(6, 1)
    data = [100, -1, 0, 0, 0, -1]
    clusters = find_frontier_clusters(data, info, buffer_cells=1)
    assert len(clusters) == 1
    assert clusters[0] == [3]
    assert 0 not in clusters[0]


def test_cell_touching_unknown_directly_excluded_from_frontier():
    # 1x6: [-1, 0, 0, 0, 0, -1]. Cells 1 and 4 touch unknown directly and
    # must never appear as frontier cells, regardless of cluster size —
    # only cells 2 and 3 (the buffer ring, one known cell removed from
    # unknown) qualify.
    info = make_info(6, 1)
    data = [-1, 0, 0, 0, 0, -1]
    clusters = find_frontier_clusters(data, info, buffer_cells=1)
    all_frontier_cells = {cell for cluster in clusters for cell in cluster}
    assert all_frontier_cells == {2, 3}
    assert 1 not in all_frontier_cells
    assert 4 not in all_frontier_cells


def test_adjacent_frontiers_form_one_cluster():
    # 6x5 grid: a 1-cell unknown border surrounds a 4x3 free interior. The
    # interior ring touching the border is excluded under the buffer rule;
    # only the two center cells (row 2, cols 2-3) are never adjacent to
    # unknown, so they are the frontier — 4-adjacent to each other, one
    # cluster of size 2.
    width, height = 6, 5
    info = make_info(width, height)
    data = [
        -1 if (r in (0, height - 1) or c in (0, width - 1)) else 0
        for r in range(height) for c in range(width)
    ]
    clusters = find_frontier_clusters(data, info, buffer_cells=1)
    assert len(clusters) == 1
    assert len(clusters[0]) == 2


def test_default_buffer_is_two_cells():
    # 1x7: [-1, 0, 0, 0, 0, 0, -1]. Cells 1,5 touch unknown (depth 0); cells 2,4
    # are the first known ring (depth 1); only cell 3 is two known cells back from
    # unknown. The default buffer_cells=2 must return exactly cell 3.
    info = make_info(7, 1)
    data = [-1, 0, 0, 0, 0, 0, -1]
    assert find_frontier_clusters(data, info) == [[3]]
    # buffer_cells=1 on the same map keeps the shallower ring (cells 2 and 4).
    one = {c for cl in find_frontier_clusters(data, info, buffer_cells=1) for c in cl}
    assert one == {2, 4}


def test_two_cell_buffer_yields_no_frontier_in_narrow_strip():
    # 1x5: [-1, 0, 0, 0, -1]. Only 3 free cells — none is 2 known cells away from
    # unknown, so the default 2-cell buffer produces no frontier at all (a free
    # region must be at least 2*buffer_cells+1 = 5 cells wide to host one).
    info = make_info(5, 1)
    data = [-1, 0, 0, 0, -1]
    assert find_frontier_clusters(data, info) == []


# --- cell_to_world ---

def test_cell_to_world_origin():
    info = make_info(10, 10, resolution=0.05)
    x, y = cell_to_world(0, info)
    assert abs(x - 0.025) < 1e-9
    assert abs(y - 0.025) < 1e-9


def test_cell_to_world_second_col():
    info = make_info(10, 10, resolution=1.0)
    x, y = cell_to_world(1, info)  # row=0, col=1
    assert abs(x - 1.5) < 1e-9
    assert abs(y - 0.5) < 1e-9


def test_cell_to_world_second_row():
    info = make_info(5, 5, resolution=1.0)
    x, y = cell_to_world(5, info)  # row=1, col=0
    assert abs(x - 0.5) < 1e-9
    assert abs(y - 1.5) < 1e-9


# --- pick_best_frontier ---

def test_pick_returns_none_when_no_clusters():
    info = make_info(5, 5)
    assert pick_best_frontier([], info, (0.0, 0.0), filters()) is None


def test_pick_skips_clusters_below_min_size():
    info = make_info(5, 1)
    cluster = [0, 1]  # size 2
    assert pick_best_frontier(
        [cluster], info, (0.0, 0.0), filters(min_frontier_size=10)
    ) is None


def test_pick_returns_nearest_centroid():
    info = make_info(10, 1, resolution=1.0)
    near = [1]   # world x=1.5
    far = [8]    # world x=8.5
    # robot at x=0, nearest is near cluster
    result = pick_best_frontier([near, far], info, (0.0, 0.0), filters())
    assert result is not None
    assert abs(result[0] - 1.5) < 1e-6


# --- preferred_dist ---

def test_pick_preferred_dist_large_returns_farthest_across_clusters():
    info = make_info(10, 1, resolution=1.0)
    near = [1]   # world x=1.5, dist=1.5
    far = [8]    # world x=8.5, dist=8.5
    result = pick_best_frontier(
        [near, far], info, (0.0, 0.0), filters(preferred_goal_distance=1000.0)
    )
    assert result is not None
    assert abs(result[0] - 8.5) < 1e-6


def test_pick_preferred_dist_large_within_single_cluster():
    info = make_info(10, 1, resolution=1.0)
    cluster = [1, 5, 8]  # world x=1.5, 5.5, 8.5
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0), filters(preferred_goal_distance=1000.0)
    )
    assert result is not None
    assert abs(result[0] - 8.5) < 1e-6


def test_pick_preferred_dist_nearest_returns_nearest():
    info = make_info(10, 1, resolution=1.0)
    near = [1]   # world x=1.5
    far = [8]    # world x=8.5
    result = pick_best_frontier(
        [near, far], info, (0.0, 0.0), filters(preferred_goal_distance=0.0)
    )
    assert result is not None
    assert abs(result[0] - 1.5) < 1e-6


def test_pick_preferred_dist_intermediate_picks_closest_to_target():
    info = make_info(10, 1, resolution=1.0)
    cluster = [1, 5, 8]  # distances 1.5, 5.5, 8.5
    # preferred_dist=5.0 → cell at x=5.5 (dist=5.5, score=0.5) wins over
    # x=1.5 (score=3.5) and x=8.5 (score=3.5)
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0), filters(preferred_goal_distance=5.0)
    )
    assert result is not None
    assert abs(result[0] - 5.5) < 1e-6


def test_pick_preferred_dist_respects_blacklist():
    info = make_info(10, 1, resolution=1.0)
    cluster = [1, 5, 8]  # world x=1.5, 5.5, 8.5
    blacklist = {(8.5, 0.5)}
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0),
        filters(blacklist_radius=0.5, preferred_goal_distance=1000.0),
        blacklist=blacklist,
    )
    assert result is not None
    assert abs(result[0] - 5.5) < 1e-6


def test_pick_preferred_dist_respects_max_dist():
    info = make_info(10, 1, resolution=1.0)
    cluster = [1, 5, 8]  # world x=1.5, 5.5, 8.5 — distances 1.5, 5.5, 8.5
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0),
        filters(max_frontier_dist=6.0, preferred_goal_distance=1000.0),
    )
    assert result is not None
    assert abs(result[0] - 5.5) < 1e-6


def test_pick_skips_blacklisted_centroid():
    info = make_info(10, 1, resolution=1.0)
    near = [1]   # centroid x=1.5
    far = [8]    # centroid x=8.5
    blacklist = {(1.5, 0.5)}
    result = pick_best_frontier(
        [near, far], info, (0.0, 0.0),
        filters(blacklist_radius=0.6), blacklist=blacklist,
    )
    assert result is not None
    assert abs(result[0] - 8.5) < 1e-6


def test_pick_returns_none_when_all_blacklisted():
    info = make_info(5, 1, resolution=1.0)
    cluster = [2]  # centroid x=2.5
    blacklist = {(2.5, 0.5)}
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0),
        filters(blacklist_radius=0.6), blacklist=blacklist,
    )
    assert result is None


def test_pick_blacklist_radius_respected():
    info = make_info(10, 1, resolution=1.0)
    cluster = [2]  # centroid x=2.5
    blacklist = {(2.1, 0.5)}  # 0.4m away — inside radius 0.5
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0),
        filters(blacklist_radius=0.5), blacklist=blacklist,
    )
    assert result is None


def test_pick_blacklist_outside_radius_not_skipped():
    info = make_info(10, 1, resolution=1.0)
    cluster = [2]  # centroid x=2.5
    blacklist = {(1.9, 0.5)}  # 0.6m away — outside radius 0.5
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0),
        filters(blacklist_radius=0.5), blacklist=blacklist,
    )
    assert result is not None


# --- max_radius filter ---

def test_max_radius_zero_disables_filter():
    info = make_info(20, 1, resolution=1.0)
    far_cluster = [15]  # centroid x=15.5, far from start (0,0)
    result = pick_best_frontier(
        [far_cluster], info, (0.0, 0.0),
        filters(max_explore_radius=0.0), start_xy=(0.5, 0.5),
    )
    assert result is not None


def test_max_radius_excludes_distant_frontier():
    info = make_info(20, 1, resolution=1.0)
    far_cluster = [15]  # centroid x=15.5
    result = pick_best_frontier(
        [far_cluster], info, (0.0, 0.0),
        filters(max_explore_radius=5.0), start_xy=(0.5, 0.5),
    )
    assert result is None


def test_max_radius_includes_near_frontier():
    info = make_info(20, 1, resolution=1.0)
    near_cluster = [2]  # centroid x=2.5, ~2m from start (0.5, 0.5)
    result = pick_best_frontier(
        [near_cluster], info, (0.0, 0.0),
        filters(max_explore_radius=5.0), start_xy=(0.5, 0.5),
    )
    assert result is not None


def test_max_radius_picks_near_over_far():
    info = make_info(20, 1, resolution=1.0)
    near_cluster = [2]   # centroid x=2.5
    far_cluster = [15]   # centroid x=15.5
    result = pick_best_frontier(
        [far_cluster, near_cluster], info, (0.0, 0.0),
        filters(max_explore_radius=5.0), start_xy=(0.5, 0.5),
    )
    assert result is not None
    assert abs(result[0] - 2.5) < 1e-6


def test_max_radius_no_start_xy_disables_filter():
    # max_radius set but no start_xy — filter must not apply
    info = make_info(20, 1, resolution=1.0)
    far_cluster = [15]
    result = pick_best_frontier(
        [far_cluster], info, (0.0, 0.0),
        filters(max_explore_radius=3.0), start_xy=None,
    )
    assert result is not None


# --- cell_to_world with non-zero origin ---

def test_cell_to_world_nonzero_origin():
    info = MapInfo(width=5, height=5, resolution=1.0, origin_x=10.0, origin_y=20.0)
    x, y = cell_to_world(0, info)
    assert abs(x - 10.5) < 1e-9
    assert abs(y - 20.5) < 1e-9


# --- find_frontier_clusters 2D diagonal adjacency ---

def test_diagonal_frontier_cells_form_one_cluster():
    # Two overlapping "plus" shapes of free cells, centered at (2,2) and
    # (3,3) on a 6x6 grid (all other cells unknown). Each plus's own center
    # is the only cell in it with all-free neighbors (its four arms each
    # touch unknown), so each center is a buffer-cell frontier — and the two
    # centers are only diagonally adjacent to each other. 8-connectivity
    # must still merge them into one cluster.
    width, height = 6, 6
    info = make_info(width, height)
    free_cells = {(1, 2), (2, 1), (2, 2), (2, 3), (3, 2), (3, 3), (3, 4), (4, 3)}
    data = [
        0 if (r, c) in free_cells else -1
        for r in range(height) for c in range(width)
    ]
    clusters = find_frontier_clusters(data, info, buffer_cells=1)
    assert len(clusters) == 1
    assert len(clusters[0]) == 2


# --- nudge_toward_robot ---

def test_nudge_pulls_goal_toward_robot():
    result = nudge_toward_robot((4.0, 0.0), (0.0, 0.0), 0.3)
    assert abs(result[0] - 3.7) < 1e-6
    assert abs(result[1] - 0.0) < 1e-6


def test_nudge_diagonal():
    # robot at origin, frontier at (3,4), dist=5, inset=0.3
    # nudged = (3 + (-3/5)*0.3, 4 + (-4/5)*0.3) = (2.82, 3.76)
    result = nudge_toward_robot((3.0, 4.0), (0.0, 0.0), 0.3)
    assert abs(result[0] - 2.82) < 1e-6
    assert abs(result[1] - 3.76) < 1e-6


def test_nudge_closer_than_inset_returns_unchanged():
    result = nudge_toward_robot((0.1, 0.0), (0.0, 0.0), 0.3)
    assert result == (0.1, 0.0)


def test_nudge_robot_at_frontier_returns_unchanged():
    result = nudge_toward_robot((1.0, 1.0), (1.0, 1.0), 0.3)
    assert result == (1.0, 1.0)


# --- pick_best_frontier nearest-cell invariant ---

def test_pick_returns_nearest_cell_not_centroid():
    # Cluster cells at x=1.5, 2.5, 8.5 → centroid ≈ 4.2, nearest cell = 1.5.
    # Verifies nearest-cell logic: if code returned centroid, result would be ~4.2.
    info = make_info(10, 1, resolution=1.0)
    cluster = [1, 2, 8]
    result = pick_best_frontier([cluster], info, (0.0, 0.0), filters())
    assert result is not None
    assert abs(result[0] - 1.5) < 1e-6


def test_pick_all_cells_under_min_dist_skips_cluster():
    # All cells in cluster are within min_dist of robot → whole cluster skipped.
    info = make_info(10, 1, resolution=1.0)
    cluster = [0, 1, 2]  # world x: 0.5, 1.5, 2.5 — all < 5.0 from robot at x=0
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0), filters(min_frontier_dist=5.0)
    )
    assert result is None


def test_pick_ring_cluster_centroid_near_robot():
    # Regression: large frontier ring surrounds robot — centroid ≈ robot position.
    # Must still return a valid goal (nearest cell beyond min_dist), not None.
    # 5x5 grid, robot at center (2,2) in world = (2.5, 2.5) at resolution=1.0.
    # Ring of frontier cells: all 4 cells at distance ~1.4 from center.
    # ring centroid = robot position → old code filtered it, new code must not.
    info = make_info(5, 5, resolution=1.0)
    # cells (1,1),(1,3),(3,1),(3,3) = indices 6,8,16,18 — equidistant from center
    ring = [6, 8, 16, 18]
    robot_xy = (2.5, 2.5)  # center of 5x5 grid at resolution 1.0
    result = pick_best_frontier(
        [ring], info, robot_xy, filters(min_frontier_dist=0.5)
    )
    assert result is not None
    # nearest cell in ring to robot: all at distance sqrt(2) ≈ 1.414, all > 0.5 min_dist
    dist = math.sqrt((result[0] - robot_xy[0]) ** 2 + (result[1] - robot_xy[1]) ** 2)
    assert dist >= 0.5


def test_pick_all_cells_over_max_dist_skips_cluster():
    # All cells farther than max_dist → cluster skipped entirely.
    info = make_info(10, 1, resolution=1.0)
    cluster = [8]  # world x=8.5, dist=8.5
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0), filters(max_frontier_dist=1.0)
    )
    assert result is None


def testfrontier_diag_reports_cells_filtered_by_max_dist():
    # Regression: a cluster entirely beyond max_dist must be counted in
    # all_cells_out_of_range, even though it passes the min_dist check (bug found
    # 2026-07-03 — the diag helper only checked min_dist, so pick_best_frontier
    # returning None due to max_dist filtering looked like an unexplained gap in
    # telemetry: large_clusters > 0 but all_cells_out_of_range reported 0).
    info = make_info(10, 1, resolution=1.0)
    cluster = [8]  # world x=8.5, dist=8.5 from robot at (0,0)
    diag = frontier_diag([cluster], info, (0.0, 0.0), min_size=1, min_dist=0.0,
                           max_dist=1.0)
    assert diag["large_clusters"] == 1
    assert diag["all_cells_out_of_range"] == 1


def test_pick_returns_cell_within_max_dist():
    # Nearest in-range cell is returned when max_dist excludes farther cells.
    info = make_info(10, 1, resolution=1.0)
    cluster = [0, 5]  # world x: 0.5 (dist 0.5), 5.5 (dist 5.5)
    result = pick_best_frontier(
        [cluster], info, (0.0, 0.0), filters(max_frontier_dist=1.0)
    )
    assert result is not None
    assert abs(result[0] - 0.5) < 1e-6
