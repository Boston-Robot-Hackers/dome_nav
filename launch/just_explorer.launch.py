#!/usr/bin/env python3
# just_explorer.launch.py — Launches only explorer_manager_node, nothing else.
# Bring your own /map and active Nav2 stack for it to send goals to. Publish an
# "exploration_start" intent on /intent to begin (see 02-doc/current.md Intent
# contract). Tuning params are explicit launch args: better_launch only forwards
# declared function args, and its click wrapper cannot parse "X | None" unions,
# so unset is signaled by -1 sentinels instead. Args left at -1 are not passed,
# keeping the node/algorithm defaults canonical. map_name only tags the
# telemetry filename -- the explorer never builds or saves a map.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def just_explorer_launch(
    map_name: str = "unknown",
    use_sim_time: bool = False,
    min_frontier_dist: float = -1.0,
    max_frontier_dist: float = -1.0,
    min_frontier_size: int = -1,
    preferred_goal_distance: float = -1.0,
    max_explore_radius: float = -1.0,
    blacklist_radius: float = -1.0,
    use_novelty_scoring: bool = False,
    w_distance: float = -1.0,
    w_novelty: float = -1.0,
    w_clearance: float = -1.0,
    robot_radius: float = -1.0,
    clearance_margin_m: float = -1.0,
):
    bl = BetterLaunch()

    # F31 scorer weights + clearance floor. w_clearance:=0.0 disables clearance
    # (the T07 baseline); left at -1 the node keeps its dataclass default (1.0).
    overrides = {
        "min_frontier_dist": min_frontier_dist,
        "max_frontier_dist": max_frontier_dist,
        "min_frontier_size": min_frontier_size,
        "preferred_goal_distance": preferred_goal_distance,
        "max_explore_radius": max_explore_radius,
        "blacklist_radius": blacklist_radius,
        "w_distance": w_distance,
        "w_novelty": w_novelty,
        "w_clearance": w_clearance,
        "robot_radius": robot_radius,
        "clearance_margin_m": clearance_margin_m,
    }
    params = {
        "map_name": map_name,
        "use_sim_time": use_sim_time,
        "use_novelty_scoring": use_novelty_scoring,
    }
    params.update({k: v for k, v in overrides.items() if v >= 0})
    print(f"[just_explorer] node params = {params}")

    bl.node(
        "dome_nav",
        "explorer_manager_node",
        name="explore_manager",
        params=params,
        ros_waittime=30.0,
    )
