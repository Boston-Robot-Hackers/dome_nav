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


def find_frontier_clusters(data: list[int], info: MapInfo) -> list[list[int]]:
    # Returns list[list[int]]: each inner list is a cluster of cell indices (flat
    # offsets into data). row = idx // width, col = idx % width. Convert to world
    # coords via cell_to_world(idx, info). A frontier cell is free (data[idx]==0)
    # with at least one 4-neighbor that is unknown (data[nb]==-1). Adjacent
    # frontier cells are grouped into clusters by 8-connectivity flood-fill.
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

    is_frontier: set[int] = set()
    for idx in range(width * height):
        if data[idx] != 0:
            continue
        for nb in neighbors4(idx):
            if data[nb] == -1:
                is_frontier.add(idx)
                break

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
) -> tuple[float, float] | None:
    rx, ry = robot_xy
    bl = blacklist or set()
    best: tuple[float, float] | None = None
    best_dist = float("inf")

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
        goal_dist = float("inf")
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
            if d < goal_dist:
                goal_dist = d
                goal = (wx, wy)
        if goal is None:
            continue
        if goal_dist < best_dist:
            best_dist = goal_dist
            best = goal

    return best


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
