#!/usr/bin/env python3
# sim_explore_node.launch.py — Starts explorer_manager_node, same config
# as sim_explore.launch.py. One piece of the manual debug stack — requires
# sim_robot.launch.py and sim_nav.launch.py already running (needs /map and an
# active Nav2 stack to send goals to). Once running, publish an
# "exploration_start" intent on /intent to begin exploring — see
# 02-doc/current.md's Intent contract table for the full payload format.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from better_launch import BetterLaunch, launch_this


# Sim-only exploration defaults, kept identical across sim_explore.launch.py,
# sim_explore_node.launch.py, and sim_nav_full.launch.py. Can't be shared via an
# imported constant: bl's CLI statically parses launch function signatures via
# AST without importing the module (better_launch/utils/introspection.py), so a
# non-literal default like `= SOME_IMPORTED_NAME` fails with "not a valid float"
# -- only literal constants written directly in the signature work.
@launch_this(ui=True)
def sim_explore_node_launch(
    map_name: str = "",
    max_explore_radius: float = 0.0,
    max_frontier_dist: float = 15.0,
    min_frontier_dist: float = 0.9,
    preferred_goal_distance: float = 2.0,
    min_frontier_size: int = 5,
):
    if not map_name:
        raise ValueError(
            "map_name is required: "
            "bl dome_nav sim_explore_node.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    bl.node(
        "dome_nav",
        "explorer_manager_node",
        name="explore_manager",
        params={
            "max_explore_radius": max_explore_radius,
            "max_frontier_dist": max_frontier_dist,
            "min_frontier_dist": min_frontier_dist,
            "preferred_goal_distance": preferred_goal_distance,
            "min_frontier_size": min_frontier_size,
            "map_name": map_name,
            "use_sim_time": True,
        },
        ros_waittime=30.0,
    )
