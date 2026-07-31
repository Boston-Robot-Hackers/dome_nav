#!/usr/bin/env python3
# frontier_algorithm.py — default frontier exploration algorithm
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from typing import TYPE_CHECKING

from visualization_msgs.msg import MarkerArray

from dome_nav.explore_context import (
    ExplorationContext,
    GoalDecision,
    RenderContext,
)
from dome_nav.explore_diagnostics import (
    format_cluster_summary,
    format_frontier_exhaustion,
)
from dome_nav.explore_markers import build_explore_markers
from dome_nav.frontier_params import (
    FrontierParams,
    declare_frontier_params,
    merge_tuning,
)
from dome_nav.frontier_explorer import (
    find_frontier_clusters,
    nudge_toward_robot,
    path_novelty_score,
    pick_best_frontier,
    frontier_diag,
)

if TYPE_CHECKING:
    from dome_nav.frontier_params import FrontierTuning


class FrontierAlgorithm:
    """Default algorithm: wraps frontier_explorer's pure functions behind the
    ExplorationAlgorithm protocol.

    Owns its own FrontierParams and its latest_clusters/latest_diag state (for the
    viz/diag hooks); neither is protocol surface. next_goal merges shared ctx.params
    with the frontier tuning.
    """

    def __init__(self, frontier_params: FrontierParams | None = None):
        self.latest_clusters: list[list[int]] = []
        self.latest_diag: dict | None = None
        self.latest_novelty: int | None = None
        self.frontier_params = frontier_params or FrontierParams()

    def declare_params(self, node):
        self.frontier_params = declare_frontier_params(node)

    def next_goal(self, ctx: ExplorationContext) -> GoalDecision:
        tuning = merge_tuning(ctx.params, self.frontier_params)
        clusters = find_frontier_clusters(
            ctx.map_data, ctx.map_info, tuning.frontier_buffer_cells
        )
        self.latest_clusters = clusters
        target = self.select_target(clusters, ctx, tuning)
        if target is None:
            self.latest_diag = frontier_diag(
                clusters,
                ctx.map_info,
                ctx.robot_xy,
                tuning.min_frontier_size,
                tuning.min_frontier_dist,
                tuning.max_frontier_dist,
            )
            # No clusters -> done; clusters but all filtered -> blocked, not done.
            if not clusters:
                return GoalDecision.done()
            return GoalDecision.blocked()
        self.latest_diag = None
        goal = nudge_toward_robot(target, ctx.robot_xy, tuning.goal_inset_m)
        return GoalDecision.new_goal(goal)

    def select_target(
        self, clusters: list[list[int]], ctx: ExplorationContext,
        tuning: "FrontierTuning",
    ) -> tuple[float, float] | None:
        """One F31 pipeline pick; stash the winner's raw novelty for telemetry.

        The registry adds the novelty scorer when use_novelty_scoring is on;
        latest_novelty holds the winner's unknown-cell count (None otherwise).
        """
        target = pick_best_frontier(
            clusters, ctx.map_info, ctx.robot_xy, tuning,
            blacklist=ctx.blacklist, start_xy=ctx.start_xy, data=ctx.map_data,
        )
        if tuning.use_novelty_scoring and target is not None:
            self.latest_novelty = path_novelty_score(
                ctx.robot_xy, target, ctx.map_data, ctx.map_info
            )
        else:
            self.latest_novelty = None
        return target

    def render_markers(self, rc: RenderContext) -> MarkerArray:
        return build_explore_markers(
            now=rc.now,
            is_exploring=rc.is_exploring,
            clusters=self.latest_clusters,
            min_frontier_size=self.frontier_params.min_frontier_size,
            map_info=rc.map_info,
            blacklist=rc.blacklist,
            goal_xy=rc.goal_xy,
        )

    def exhaustion_report(self, rc: RenderContext) -> str:
        tuning = merge_tuning(rc.params, self.frontier_params)
        return format_frontier_exhaustion(
            self.latest_clusters, rc.map_info, rc.robot_xy,
            tuning, rc.blacklist, rc.patience,
        )

    def failure_report(self, rc: RenderContext) -> str:
        return format_cluster_summary(self.latest_clusters, rc.map_info)

    def telemetry_extra(self) -> dict:
        diag = self.latest_diag or {}
        extra = {"raw_clusters": len(self.latest_clusters), **diag}
        if self.latest_novelty is not None:
            extra["novelty_score"] = self.latest_novelty
        return extra

    def session_params(self) -> dict:
        # Frontier tuning logged at session start, merged blindly by the node.
        fp = self.frontier_params
        return {
            "min_frontier_dist": fp.min_frontier_dist,
            "max_frontier_dist": fp.max_frontier_dist,
            "goal_inset": fp.goal_inset_m,
            "min_frontier_size": fp.min_frontier_size,
            "use_novelty_scoring": fp.use_novelty_scoring,
            "preferred_goal_distance": fp.preferred_goal_distance,
        }
