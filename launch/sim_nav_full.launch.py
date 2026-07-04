#!/usr/bin/env python3
# sim_nav_full.launch.py — single-command full sim stack, composed from the
# existing single-purpose sim_*.launch.py files via bl.include() rather than
# duplicating their logic (as sim_explore.launch.py currently does). Includes,
# in the dependency order established during F13 T04 debugging: sim_robot
# (Gazebo/bridge/RSP/laser TF), sim_slam (must be up before Nav2 so the "map"
# TF frame exists), sim_nav2, then sim_explore_node. RViz is intentionally not
# included — sim_rviz.launch.py stays a separate, optional window.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def sim_nav_full_launch(
    map_name: str = "",
    max_explore_radius: float = 0.0,
    max_frontier_dist: float = 3.0,
    prefer_farthest: bool = True,
):
    if not map_name:
        raise ValueError(
            "map_name is required: bl dome_nav sim_nav_full.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    bl.include("dome_nav", "sim_robot.launch.py")
    bl.include("dome_nav", "sim_slam.launch.py")
    bl.include("dome_nav", "sim_nav2.launch.py")
    bl.include("dome_nav", "sim_explore_node.launch.py")
