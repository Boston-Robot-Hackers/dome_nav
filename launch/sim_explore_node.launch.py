#!/usr/bin/env python3
# sim_explore_node.launch.py — Starts pluggable_explore_manager_node, same config
# as sim_explore.launch.py. One piece of the manual debug stack — requires
# sim_robot.launch.py and sim_nav.launch.py already running (needs /map and an
# active Nav2 stack to send goals to). Once running, publish an
# "exploration_start" intent on /intent to begin exploring — see
# 02-doc/current.md's Intent contract table for the full payload format.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def sim_explore_node_launch(
    map_name: str = "",
    max_explore_radius: float = 0.0,
    max_frontier_dist: float = 3.0,
    prefer_farthest: bool = True,
):
    if not map_name:
        raise ValueError(
            "map_name is required: "
            "bl dome_nav sim_explore_node.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    bl.node(
        "dome_nav",
        "pluggable_explore_manager_node",
        name="explore_manager",
        params={
            "max_explore_radius": max_explore_radius,
            "max_frontier_dist": max_frontier_dist,
            "prefer_farthest": prefer_farthest,
            "map_name": map_name,
            "use_sim_time": True,
        },
        ros_waittime=30.0,
    )
