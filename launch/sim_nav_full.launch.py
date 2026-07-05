#!/usr/bin/env python3
# sim_nav_full.launch.py — single-command full sim stack, composed from the
# existing single-purpose sim_*.launch.py files via bl.include() rather than
# duplicating their logic (as sim_explore.launch.py currently does). Includes,
# in the dependency order established during F13 T04 debugging: sim_robot
# (Gazebo/bridge/RSP/laser TF), sim_slam (must be up before Nav2 so the "map"
# TF frame exists), sim_nav2, then sim_explore_node. RViz is intentionally not
# included — sim_rviz.launch.py stays a separate, optional window.
# Also starts slam_manager_node directly (not via an include, since none of
# the split files own it) so maps built through this single-command launch
# actually get persisted to ~/.dome/slam_maps/ like sim_explore.launch.py's do.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home


@launch_this(ui=True)
def sim_nav_full_launch(
    map_name: str = "",
    max_explore_radius: float = 0.0,
    max_frontier_dist: float = 3.0,
    prefer_farthest: bool = True,
    min_frontier_size: int = 1,
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

    slam_map_path = os.path.join(dome_home(), "slam_maps", map_name)
    bl.node(
        "dome_nav",
        "slam_manager_node",
        name="slam_manager",
        params={
            "map_persist_path": slam_map_path,
            "use_sim_time": True,
            "save_period_sec": 120.0,
        },
        ros_waittime=30.0,
        lifecycle_waittime=None,
    )
