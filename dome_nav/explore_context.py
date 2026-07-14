#!/usr/bin/env python3
# explore_context.py — data types and protocol for pluggable exploration algorithms
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dataclasses import dataclass
from typing import Protocol

from dome_nav.frontier_explorer import MapInfo


@dataclass
class ExploreParams:
    min_frontier_size: int = 15
    blacklist_radius: float = 0.5
    min_frontier_dist: float = 1.3
    max_frontier_dist: float = 0.0
    goal_inset_m: float = 0.3
    max_explore_radius: float = 0.0
    preferred_goal_distance: float = 1.0
    prefer_farthest: bool = False  # deprecated: use preferred_goal_distance
    # Known-cell rings between a frontier goal and the unknown boundary. 2 keeps
    # goals two confirmed-known cells inside the mapped edge (see
    # find_frontier_clusters); 1 is the original single-buffer behaviour.
    frontier_buffer_cells: int = 2


@dataclass
class ExplorationContext:
    map_data: list[int]
    map_info: MapInfo
    robot_xy: tuple[float, float]
    blacklist: set[tuple[float, float]]
    start_xy: tuple[float, float] | None
    params: ExploreParams


class ExplorationAlgorithm(Protocol):
    latest_clusters: list[list[int]]
    latest_diag: dict | None

    def next_goal(self, ctx: ExplorationContext) -> tuple[float, float] | None: ...
