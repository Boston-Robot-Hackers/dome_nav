#!/usr/bin/env python3
# frontier_params.py — frontier-algorithm-owned tuning params and ROS declaration
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from dataclasses import dataclass

from dome_nav.explore_context import ExploreParams


@dataclass
class FrontierParams:
    # Frontier-only tuning, owned and declared by FrontierAlgorithm (not the
    # manager node). These moved out of ExploreParams in F23 T03 because they are
    # meaningless to a non-frontier strategy. The frontier algorithm self-declares
    # them as ROS params (see declare_frontier_params) so they stay yaml/launch
    # settable without the node knowing their names.
    min_frontier_size: int = 15
    min_frontier_dist: float = 1.3
    max_frontier_dist: float = 0.0
    goal_inset_m: float = 0.3
    # Known-cell rings between a frontier goal and the unknown boundary. 2 keeps
    # goals two confirmed-known cells inside the mapped edge (see
    # find_frontier_clusters); 1 is the original single-buffer behaviour.
    frontier_buffer_cells: int = 2
    prefer_farthest: bool = False  # deprecated: use preferred_goal_distance


@dataclass
class FrontierTuning:
    # Combined read-only view the frontier pure functions and diagnostics consume:
    # the shared/session fields (from ExploreParams) the frontier code still needs,
    # merged with the frontier-owned fields. Assembled per tick by FrontierAlgorithm
    # from ctx.params + its FrontierParams, so neither dataclass has to carry the
    # other's fields.
    min_frontier_size: int
    blacklist_radius: float
    min_frontier_dist: float
    max_frontier_dist: float
    goal_inset_m: float
    max_explore_radius: float
    preferred_goal_distance: float
    frontier_buffer_cells: int
    prefer_farthest: bool


def merge_tuning(shared: ExploreParams, frontier: FrontierParams) -> FrontierTuning:
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
    )


def declare_frontier_params(node) -> FrontierParams:
    # Self-declare the frontier tuning as ROS params in the node's namespace and
    # read them back into a FrontierParams. Chosen over a node-driven schema so no
    # frontier param name ever appears in explorer_manager_node.py (F23 T04). The
    # node calls this once via the algorithm's declare_params hook.
    defaults = FrontierParams()
    node.declare_parameter("min_frontier_size", defaults.min_frontier_size)
    node.declare_parameter("min_frontier_dist", defaults.min_frontier_dist)
    node.declare_parameter("max_frontier_dist", defaults.max_frontier_dist)
    node.declare_parameter("goal_inset_m", defaults.goal_inset_m)
    node.declare_parameter("frontier_buffer_cells", defaults.frontier_buffer_cells)
    node.declare_parameter("prefer_farthest", defaults.prefer_farthest)  # deprecated
    return FrontierParams(
        min_frontier_size=node.get_parameter("min_frontier_size").value,
        min_frontier_dist=node.get_parameter("min_frontier_dist").value,
        max_frontier_dist=node.get_parameter("max_frontier_dist").value,
        goal_inset_m=node.get_parameter("goal_inset_m").value,
        frontier_buffer_cells=node.get_parameter("frontier_buffer_cells").value,
        prefer_farthest=node.get_parameter("prefer_farthest").value,
    )
