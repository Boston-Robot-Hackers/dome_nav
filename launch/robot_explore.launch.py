#!/usr/bin/env python3
# robot_explore.launch.py — Mode A stack + frontier exploration for autonomous
# map building
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home


@launch_this(ui=True)
def robot_explore_launch(
    use_sim_time: str = "false",
    map_name: str = "",
    max_explore_radius: float = 0.0,
):
    if not map_name:
        raise ValueError(
            "map_name is required: bl robot_explore.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    home = dome_home()
    slam_map_path = os.path.join(home, "slam_maps", map_name)
    os.makedirs(home, exist_ok=True)

    pkg = get_package_share_directory("dome_nav")

    slam_config = os.path.join(pkg, "config", "mapper_params_online_async.yaml")

    # Full standalone config, loaded verbatim -- no patch chain.
    nav2_config = os.path.join(pkg, "config", "nav2_params_explore_real.yaml")

    bl.include("slam_toolbox", "online_async_launch.py",
        **{"slam_params_file": slam_config, "use_sim_time": use_sim_time})

    bl.include("nav2_bringup", "navigation_launch.py",
        **{"params_file": nav2_config, "use_sim_time": use_sim_time})

    bl.node(
        "dome_nav",
        "slam_manager_node",
        name="slam_manager",
        params={"map_persist_path": slam_map_path},
        ros_waittime=30.0,
        lifecycle_waittime=None,
    )

    # Same explorer node as the sim stack (explorer_manager_node), differing
    # only by parameter values -- sim and real share one code path. The values below
    # are the explicit real-robot explore settings; several intentionally differ from
    # ExploreParams' dataclass defaults (min_frontier_dist 0.5 vs 1.3,
    # preferred_goal_distance 2.0 vs 1.0, min_frontier_size 10 vs 15,
    # frontier_buffer_cells 0 vs 2). max_frontier_dist 0.0 = unlimited. Note
    # blacklist_radius (0.5) and goal_inset_m (0.3) are not exposed here -- they use
    # ExploreParams defaults and can only be changed in code. Sim launch files set
    # their own values for the simulated worlds.
    bl.node(
        "dome_nav",
        "explorer_manager_node",
        name="explore_manager",
        params={
            "max_explore_radius": max_explore_radius,
            "max_frontier_dist": 0.0,
            "min_frontier_dist": 0.5,
            "preferred_goal_distance": 2.0,
            "frontier_buffer_cells": 0,
            "min_frontier_size": 10,
            "map_name": map_name,
            "use_sim_time": use_sim_time == "true",
        },
        ros_waittime=30.0,
    )
