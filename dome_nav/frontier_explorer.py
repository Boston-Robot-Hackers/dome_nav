#!/usr/bin/env python3
# frontier_explorer.py — pure Python frontier detection from OccupancyGrid data
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import math
from dataclasses import dataclass


@dataclass
class MapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


def find_frontier_clusters(
    data: list[int], info: MapInfo, buffer_cells: int = 2
) -> list[list[int]]:
    # Returns list[list[int]]: each inner list is a cluster of cell indices (flat
    # offsets into data). row = idx // width, col = idx % width. Convert to world
    # coords via cell_to_world(idx, info).
    #
    # A frontier cell is free (data[idx]==0) sitting exactly `buffer_cells` known
    # cells inside the boundary with the unknown region. The boundary ring is the
    # free cells that directly touch an unknown 4-neighbor; each successive ring
    # is the free cells 4-adjacent to the previous one. The last ring is the
    # frontier, so every candidate goal is `buffer_cells` confirmed-known cells
    # away from the ragged known/unknown edge — where Nav2's planners have
    # historically been unreliable (the NavFn "legal potential" bug, TF13 T04m),
    # where costmap geometry is most ambiguous, and (buffer_cells>=2) far enough
    # inside the mapped area to survive the multi-cell seam between the SLAM /map
    # the frontier detector reads and the smaller global costmap the planner uses
    # — the worldToMap "goal outside map" failure. buffer_cells=1 reproduces the
    # original single-buffer-ring behaviour. Note: a free region narrower than
    # 2*buffer_cells+1 cells has no cell far enough from unknown and yields no
    # frontier there. Adjacent frontier cells are grouped into clusters by
    # 8-connectivity flood-fill.
    width, height = info.width, info.height

    def neighbors4(idx: int):
        r, c = divmod(idx, width)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if 0 <= nr < height and 0 <= nc < width:
                yield nr * width + nc

    def neighbors8(idx: int):
        r, c = divmod(idx, width)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < height and 0 <= nc < width:
                    yield nr * width + nc

    # Boundary ring (depth 0): free cells directly adjacent to unknown.
    boundary: set[int] = set()
    for idx in range(width * height):
        if data[idx] != 0:
            continue
        for nb in neighbors4(idx):
            if data[nb] == -1:
                boundary.add(idx)
                break

    # Walk `buffer_cells` rings of free cells inward from the boundary. Each new
    # ring is the free cells 4-adjacent to the previous ring that no shallower
    # ring already claimed; the last ring reached is the frontier. If a ring runs
    # out of room (narrow free strip) the frontier set is empty there.
    claimed: set[int] = set(boundary)
    ring: set[int] = boundary
    is_frontier: set[int] = set()
    for _ in range(max(1, buffer_cells)):
        next_ring: set[int] = set()
        for idx in ring:
            for nb in neighbors4(idx):
                if data[nb] == 0 and nb not in claimed:
                    next_ring.add(nb)
        claimed |= next_ring
        ring = next_ring
        is_frontier = next_ring

    visited: set[int] = set()
    clusters: list[list[int]] = []
    for seed in is_frontier:
        if seed in visited:
            continue
        cluster: list[int] = []
        stack = [seed]
        while stack:
            cell = stack.pop()
            if cell in visited or cell not in is_frontier:
                continue
            visited.add(cell)
            cluster.append(cell)
            for nb in neighbors8(cell):
                if nb not in visited and nb in is_frontier:
                    stack.append(nb)
        clusters.append(cluster)
    return clusters


def cell_to_world(idx: int, info: MapInfo) -> tuple[float, float]:
    r, c = divmod(idx, info.width)
    x = info.origin_x + (c + 0.5) * info.resolution
    y = info.origin_y + (r + 0.5) * info.resolution
    return (x, y)


# Returns the nearest non-blacklisted frontier cell (not centroid) beyond min_dist.
# Using the nearest cell rather than centroid avoids the ring-cluster problem: a
# large frontier surrounding the robot has centroid ≈ robot position, but individual
# cells are at the map boundary where the robot actually needs to go.
# Centroid is still used for max_radius filtering (cluster-level position proxy).
# Blacklist is checked per-cell so only visited cells are excluded, not entire clusters.

def pick_best_frontier(
    clusters: list[list[int]],
    info: MapInfo,
    robot_xy: tuple[float, float],
    min_size: int = 10,
    blacklist: set[tuple[float, float]] | None = None,
    blacklist_radius: float = 0.5,
    max_radius: float = 0.0,
    start_xy: tuple[float, float] | None = None,
    min_dist: float = 0.0,
    max_dist: float = 0.0,
    prefer_farthest: bool = False,
) -> tuple[float, float] | None:
    rx, ry = robot_xy
    bl = blacklist or set()
    best: tuple[float, float] | None = None
    best_dist = -1.0 if prefer_farthest else float("inf")

    for cluster in clusters:
        if len(cluster) < min_size:
            continue
        if max_radius > 0.0 and start_xy is not None:
            cx = sum(cell_to_world(i, info)[0] for i in cluster) / len(cluster)
            cy = sum(cell_to_world(i, info)[1] for i in cluster) / len(cluster)
            sx, sy = start_xy
            if math.sqrt((cx - sx) ** 2 + (cy - sy) ** 2) > max_radius:
                continue
        goal: tuple[float, float] | None = None
        goal_dist = -1.0 if prefer_farthest else float("inf")
        for cell_idx in cluster:
            wx, wy = cell_to_world(cell_idx, info)
            too_close = any(
                math.sqrt((wx - bx) ** 2 + (wy - by) ** 2) < blacklist_radius
                for bx, by in bl
            )
            if too_close:
                continue
            d = math.sqrt((wx - rx) ** 2 + (wy - ry) ** 2)
            if min_dist > 0.0 and d < min_dist:
                continue
            if max_dist > 0.0 and d > max_dist:
                continue
            is_better = d > goal_dist if prefer_farthest else d < goal_dist
            if is_better:
                goal_dist = d
                goal = (wx, wy)
        if goal is None:
            continue
        is_better = goal_dist > best_dist if prefer_farthest else goal_dist < best_dist
        if is_better:
            best_dist = goal_dist
            best = goal

    return best


def frontier_diag(
    clusters: list[list[int]],
    info: MapInfo,
    robot_xy: tuple[float, float],
    min_size: int,
    min_dist: float,
    max_dist: float = 0.0,
) -> dict:
    # Returns filter-stage counts for telemetry. Cheap extra pass; only called
    # when pick_best_frontier returns None so normal-path performance is unaffected.
    rx, ry = robot_xy
    too_small = sum(1 for c in clusters if len(c) < min_size)
    large = [c for c in clusters if len(c) >= min_size]
    all_out_of_range = 0
    for cluster in large:
        if all(
            cell_out_of_range(cell_to_world(i, info), robot_xy, min_dist, max_dist)
            for i in cluster
        ):
            all_out_of_range += 1
    return {
        "too_small": too_small,
        "large_clusters": len(large),
        "all_cells_out_of_range": all_out_of_range,
    }


def cell_out_of_range(
    cell_xy: tuple[float, float],
    robot_xy: tuple[float, float],
    min_dist: float,
    max_dist: float,
) -> bool:
    d = math.sqrt((cell_xy[0] - robot_xy[0]) ** 2 + (cell_xy[1] - robot_xy[1]) ** 2)
    return (min_dist > 0.0 and d < min_dist) or (max_dist > 0.0 and d > max_dist)


def nudge_toward_robot(
    xy: tuple[float, float], robot_xy: tuple[float, float], inset_m: float
) -> tuple[float, float]:
    # Pull xy toward robot_xy by inset_m. Keeps the nav goal inside the costmap
    # boundary rather than on the unknown-cell edge (avoids Nav2 worldToMap errors).
    dx = robot_xy[0] - xy[0]
    dy = robot_xy[1] - xy[1]
    dist = math.sqrt(dx * dx + dy * dy)
    if dist < inset_m:
        return xy
    scale = inset_m / dist
    return (xy[0] + dx * scale, xy[1] + dy * scale)
