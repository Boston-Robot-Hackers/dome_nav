#!/usr/bin/env python3
# frontier_algorithm.py — default frontier exploration algorithm
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dome_nav.explore_context import ExplorationContext
from dome_nav.frontier_explorer import (
    find_frontier_clusters,
    nudge_toward_robot,
    pick_best_frontier,
    _frontier_diag,
)


class FrontierAlgorithm:
    # Default exploration algorithm. Wraps the pure functions in
    # frontier_explorer.py behind the ExplorationAlgorithm protocol.

    def __init__(self):
        self.latest_clusters: list[list[int]] = []
        self.latest_diag: dict | None = None

    def next_goal(
        self, ctx: ExplorationContext
    ) -> tuple[float, float] | None:
        clusters = find_frontier_clusters(ctx.map_data, ctx.map_info)
        self.latest_clusters = clusters
        target = pick_best_frontier(
            clusters,
            ctx.map_info,
            ctx.robot_xy,
            min_size=ctx.params.min_frontier_size,
            blacklist=ctx.blacklist,
            blacklist_radius=ctx.params.blacklist_radius,
            max_radius=ctx.params.max_explore_radius,
            start_xy=ctx.start_xy,
            min_dist=ctx.params.min_frontier_dist,
            max_dist=ctx.params.max_frontier_dist,
            prefer_farthest=ctx.params.prefer_farthest,
        )
        if target is None:
            self.latest_diag = _frontier_diag(
                clusters,
                ctx.map_info,
                ctx.robot_xy,
                ctx.params.min_frontier_size,
                ctx.params.min_frontier_dist,
                ctx.params.max_frontier_dist,
            )
            return None
        self.latest_diag = None
        return nudge_toward_robot(target, ctx.robot_xy, ctx.params.goal_inset_m)
