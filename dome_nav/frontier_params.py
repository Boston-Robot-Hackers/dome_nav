#!/usr/bin/env python3
# frontier_params.py — frontier-algorithm-owned tuning params and ROS declaration
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dataclasses import dataclass

from dome_nav.explore_context import ExploreParams


@dataclass
class FrontierParams:
    """Frontier-only tuning, owned and self-declared by FrontierAlgorithm."""
    min_frontier_size: int = 15
    min_frontier_dist: float = 1.3
    max_frontier_dist: float = 0.0
    goal_inset_m: float = 0.3
    frontier_buffer_cells: int = 2  # known-cell rings inside the unknown boundary
    prefer_farthest: bool = False  # deprecated: use preferred_goal_distance
    use_novelty_scoring: bool = False  # opt-in: adds the novelty scorer (F31 pipeline)
    novelty_top_n: int = 5  # deprecated no-op: the two-stage short-list retired in F31
    # F31 scorer weights (all scorers normalized [0,1] per cycle, so comparable).
    w_distance: float = 1.0  # distance-to-preferred scorer weight
    w_novelty: float = 1.0   # novelty weight (only when use_novelty_scoring)
    w_clearance: float = 1.0  # clearance scorer weight; 0 disables F31 clearance
    robot_radius: float = 0.17  # R_inscribed for the clearance floor (real default)
    clearance_margin_m: float = 0.05  # floor = robot_radius + this; keep small


@dataclass
class FrontierTuning:
    """Shared + frontier fields merged per tick for pure functions and diagnostics."""
    min_frontier_size: int
    blacklist_radius: float
    min_frontier_dist: float
    max_frontier_dist: float
    goal_inset_m: float
    max_explore_radius: float
    preferred_goal_distance: float
    frontier_buffer_cells: int
    prefer_farthest: bool
    use_novelty_scoring: bool
    novelty_top_n: int
    w_distance: float
    w_novelty: float
    w_clearance: float
    robot_radius: float
    clearance_margin_m: float


def merge_tuning(shared: ExploreParams, frontier: FrontierParams) -> FrontierTuning:
    """Merge shared and frontier params into one per-tick FrontierTuning.

    Enforces blacklist_radius > goal_inset_m at this boundary: exclusion coords are
    stored post-nudge but filtered against raw cells goal_inset_m apart, so a radius
    at or below the inset would let an excluded cell reselect every tick.
    """
    if shared.blacklist_radius <= frontier.goal_inset_m:
        raise ValueError(
            f"blacklist_radius ({shared.blacklist_radius}) must exceed "
            f"goal_inset_m ({frontier.goal_inset_m}); exclusion is stored "
            "post-nudge but filtered against raw cells."
        )
    # Deprecated prefer_farthest maps to farthest-first selection: preferred goal
    # distance becomes max_frontier_dist (or a large sentinel when unlimited).
    preferred = shared.preferred_goal_distance
    if frontier.prefer_farthest:
        has_max = frontier.max_frontier_dist > 0.0
        preferred = frontier.max_frontier_dist if has_max else 1000.0
    return FrontierTuning(
        min_frontier_size=frontier.min_frontier_size,
        blacklist_radius=shared.blacklist_radius,
        min_frontier_dist=frontier.min_frontier_dist,
        max_frontier_dist=frontier.max_frontier_dist,
        goal_inset_m=frontier.goal_inset_m,
        max_explore_radius=shared.max_explore_radius,
        preferred_goal_distance=preferred,
        frontier_buffer_cells=frontier.frontier_buffer_cells,
        prefer_farthest=frontier.prefer_farthest,
        use_novelty_scoring=frontier.use_novelty_scoring,
        novelty_top_n=frontier.novelty_top_n,
        w_distance=frontier.w_distance,
        w_novelty=frontier.w_novelty,
        w_clearance=frontier.w_clearance,
        robot_radius=frontier.robot_radius,
        clearance_margin_m=frontier.clearance_margin_m,
    )


def declare_frontier_params(node) -> FrontierParams:
    """Declare the frontier tuning as ROS params in the node's namespace and read back.

    Keeps every frontier param name out of the node itself.
    """
    defaults = FrontierParams()
    node.declare_parameter("min_frontier_size", defaults.min_frontier_size)
    node.declare_parameter("min_frontier_dist", defaults.min_frontier_dist)
    node.declare_parameter("max_frontier_dist", defaults.max_frontier_dist)
    node.declare_parameter("goal_inset_m", defaults.goal_inset_m)
    node.declare_parameter("frontier_buffer_cells", defaults.frontier_buffer_cells)
    node.declare_parameter("prefer_farthest", defaults.prefer_farthest)  # deprecated
    node.declare_parameter("use_novelty_scoring", defaults.use_novelty_scoring)
    node.declare_parameter("novelty_top_n", defaults.novelty_top_n)
    node.declare_parameter("w_distance", defaults.w_distance)
    node.declare_parameter("w_novelty", defaults.w_novelty)
    node.declare_parameter("w_clearance", defaults.w_clearance)
    node.declare_parameter("robot_radius", defaults.robot_radius)
    node.declare_parameter("clearance_margin_m", defaults.clearance_margin_m)
    return FrontierParams(
        min_frontier_size=node.get_parameter("min_frontier_size").value,
        min_frontier_dist=node.get_parameter("min_frontier_dist").value,
        max_frontier_dist=node.get_parameter("max_frontier_dist").value,
        goal_inset_m=node.get_parameter("goal_inset_m").value,
        frontier_buffer_cells=node.get_parameter("frontier_buffer_cells").value,
        prefer_farthest=node.get_parameter("prefer_farthest").value,
        use_novelty_scoring=node.get_parameter("use_novelty_scoring").value,
        novelty_top_n=node.get_parameter("novelty_top_n").value,
        w_distance=node.get_parameter("w_distance").value,
        w_novelty=node.get_parameter("w_novelty").value,
        w_clearance=node.get_parameter("w_clearance").value,
        robot_radius=node.get_parameter("robot_radius").value,
        clearance_margin_m=node.get_parameter("clearance_margin_m").value,
    )
