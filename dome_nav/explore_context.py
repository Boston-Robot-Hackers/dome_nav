#!/usr/bin/env python3
# explore_context.py — data types and protocol for pluggable exploration algorithms
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dataclasses import dataclass
from typing import Protocol

from dome_nav.frontier_explorer import MapInfo


@dataclass
class ExploreParams:
    min_frontier_size: int = 10
    blacklist_radius: float = 0.5
    # Checked on the raw frontier cell, before goal_inset_m pulls the actual Nav2
    # goal closer to the robot — so the real floor on the sent goal's distance is
    # min_frontier_dist - goal_inset_m. 1.3 - 0.3 = 1.0 m: the actual Nav2 goal is
    # never closer than 1.0 m from the robot.
    min_frontier_dist: float = 1.3
    max_frontier_dist: float = 0.0
    goal_inset_m: float = 0.3
    max_explore_radius: float = 0.0


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
