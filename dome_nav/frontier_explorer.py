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
    # Frontier cell: free (0) with at least one unknown (-1) 4-neighbor.
    # Clusters built with 8-connectivity flood-fill.
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


# blacklist_radius: any frontier centroid within this distance of a blacklisted
# position is skipped — prevents retrying goals Nav2 already failed to reach.
# max_radius / start_xy: if max_radius > 0, frontiers beyond that distance from
# start_xy are skipped — limits map to a circle around the exploration origin.
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
        cx = sum(cell_to_world(i, info)[0] for i in cluster) / len(cluster)
        cy = sum(cell_to_world(i, info)[1] for i in cluster) / len(cluster)
        if any(math.sqrt((cx - bx) ** 2 + (cy - by) ** 2) < blacklist_radius for bx, by in bl):
            continue
        if max_radius > 0.0 and start_xy is not None:
            sx, sy = start_xy
            if math.sqrt((cx - sx) ** 2 + (cy - sy) ** 2) > max_radius:
                continue
        dist = math.sqrt((cx - rx) ** 2 + (cy - ry) ** 2)
        if min_dist > 0.0 and dist < min_dist:
            continue
        if dist < best_dist:
            best_dist = dist
            best = (cx, cy)

    return best
