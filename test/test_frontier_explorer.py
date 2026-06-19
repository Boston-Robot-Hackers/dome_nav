#!/usr/bin/env python3
# test_frontier_explorer.py — unit tests for FrontierExplorer pure logic
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import math
import pytest
from dome_nav.frontier_explorer import MapInfo, find_frontier_clusters, cell_to_world, pick_best_frontier


def make_info(width: int, height: int, resolution: float = 1.0) -> MapInfo:
    return MapInfo(width=width, height=height, resolution=resolution, origin_x=0.0, origin_y=0.0)


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
    # 3x3: center is free, surrounded by unknown on top edge
    # Row-major: idx = row*width + col
    # Layout (3x3): all unknown except cell (1,1) = free, (0,1) = unknown neighbor
    info = make_info(3, 3)
    data = [-1] * 9
    data[4] = 0  # center cell (row=1, col=1) = free, neighbors include unknowns
    clusters = find_frontier_clusters(data, info)
    assert len(clusters) == 1
    assert clusters[0] == [4]


def test_two_separate_clusters():
    # 5x1 map: [free, unknown, unknown, unknown, free]
    # Cell 0 is free, neighbor cell 1 is unknown → frontier
    # Cell 4 is free, neighbor cell 3 is unknown → frontier
    # They are not 8-adjacent so two clusters
    info = make_info(5, 1)
    data = [0, -1, -1, -1, 0]
    clusters = find_frontier_clusters(data, info)
    assert len(clusters) == 2
    total_cells = sum(len(c) for c in clusters)
    assert total_cells == 2


def test_occupied_cell_not_frontier():
    info = make_info(3, 1)
    data = [100, -1, 0]  # cell 2 free, neighbor cell 1 unknown → frontier; cell 0 occupied
    clusters = find_frontier_clusters(data, info)
    assert len(clusters) == 1
    assert 2 in clusters[0]
    assert 0 not in clusters[0]


def test_adjacent_frontiers_form_one_cluster():
    # 1x4: [-1, free, free, -1] → cells 1 and 2 both touch unknown neighbors, are 8-adjacent → one cluster
    info = make_info(4, 1)
    data = [-1, 0, 0, -1]
    clusters = find_frontier_clusters(data, info)
    assert len(clusters) == 1
    assert len(clusters[0]) == 2


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
    assert pick_best_frontier([], info, (0.0, 0.0)) is None


def test_pick_skips_clusters_below_min_size():
    info = make_info(5, 1)
    cluster = [0, 1]  # size 2
    assert pick_best_frontier([cluster], info, (0.0, 0.0), min_size=10) is None


def test_pick_returns_nearest_centroid():
    info = make_info(10, 1, resolution=1.0)
    near = [1]   # world x=1.5
    far = [8]    # world x=8.5
    # robot at x=0, nearest is near cluster
    result = pick_best_frontier([near, far], info, (0.0, 0.0), min_size=1)
    assert result is not None
    assert abs(result[0] - 1.5) < 1e-6


def test_pick_skips_blacklisted_centroid():
    info = make_info(10, 1, resolution=1.0)
    near = [1]   # centroid x=1.5
    far = [8]    # centroid x=8.5
    blacklist = {(1.5, 0.5)}
    result = pick_best_frontier([near, far], info, (0.0, 0.0), min_size=1, blacklist=blacklist, blacklist_radius=0.6)
    assert result is not None
    assert abs(result[0] - 8.5) < 1e-6


def test_pick_returns_none_when_all_blacklisted():
    info = make_info(5, 1, resolution=1.0)
    cluster = [2]  # centroid x=2.5
    blacklist = {(2.5, 0.5)}
    result = pick_best_frontier([cluster], info, (0.0, 0.0), min_size=1, blacklist=blacklist, blacklist_radius=0.6)
    assert result is None


def test_pick_blacklist_radius_respected():
    info = make_info(10, 1, resolution=1.0)
    cluster = [2]  # centroid x=2.5
    # blacklist point 0.4m away — inside default radius 0.5
    blacklist = {(2.1, 0.5)}
    result = pick_best_frontier([cluster], info, (0.0, 0.0), min_size=1, blacklist=blacklist, blacklist_radius=0.5)
    assert result is None


def test_pick_blacklist_outside_radius_not_skipped():
    info = make_info(10, 1, resolution=1.0)
    cluster = [2]  # centroid x=2.5
    # blacklist point 0.6m away — outside radius 0.5
    blacklist = {(1.9, 0.5)}
    result = pick_best_frontier([cluster], info, (0.0, 0.0), min_size=1, blacklist=blacklist, blacklist_radius=0.5)
    assert result is not None


# --- max_radius filter ---

def test_max_radius_zero_disables_filter():
    info = make_info(20, 1, resolution=1.0)
    far_cluster = [15]  # centroid x=15.5, far from start (0,0)
    result = pick_best_frontier([far_cluster], info, (0.0, 0.0), min_size=1, max_radius=0.0, start_xy=(0.5, 0.5))
    assert result is not None


def test_max_radius_excludes_distant_frontier():
    info = make_info(20, 1, resolution=1.0)
    far_cluster = [15]  # centroid x=15.5
    result = pick_best_frontier([far_cluster], info, (0.0, 0.0), min_size=1, max_radius=5.0, start_xy=(0.5, 0.5))
    assert result is None


def test_max_radius_includes_near_frontier():
    info = make_info(20, 1, resolution=1.0)
    near_cluster = [2]  # centroid x=2.5, ~2m from start (0.5, 0.5)
    result = pick_best_frontier([near_cluster], info, (0.0, 0.0), min_size=1, max_radius=5.0, start_xy=(0.5, 0.5))
    assert result is not None


def test_max_radius_picks_near_over_far():
    info = make_info(20, 1, resolution=1.0)
    near_cluster = [2]   # centroid x=2.5
    far_cluster = [15]   # centroid x=15.5
    result = pick_best_frontier(
        [far_cluster, near_cluster], info, (0.0, 0.0),
        min_size=1, max_radius=5.0, start_xy=(0.5, 0.5)
    )
    assert result is not None
    assert abs(result[0] - 2.5) < 1e-6


def test_max_radius_no_start_xy_disables_filter():
    # max_radius set but no start_xy — filter must not apply
    info = make_info(20, 1, resolution=1.0)
    far_cluster = [15]
    result = pick_best_frontier([far_cluster], info, (0.0, 0.0), min_size=1, max_radius=3.0, start_xy=None)
    assert result is not None
