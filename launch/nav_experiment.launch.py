#!/usr/bin/env python3
# nav_experiment.launch.py -- experiment harness. slam_toolbox + Nav2, and
# OPTIONALLY the explorer stack (slam_manager + explore_manager) when --map_name
# is given. Takes the two config yamls as arguments so experiments swap configs
# without editing code. Assumes the driver stack (tf, laser, odom, base) runs
# separately.
#
#   # nav only (no explorer):
#   bl dome_nav nav_experiment.launch.py \
#       --slam_config <abs path to slam yaml> \
#       --nav2_config <abs path to nav2 yaml>
#
#   # with explorer:
#   bl dome_nav nav_experiment.launch.py \
#       --slam_config <slam yaml> --nav2_config <nav2 yaml> \
#       --map_name <name>
#
# See experiment.md for the log of what was tried.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home


@launch_this(ui=True)
def nav_experiment_launch(
    slam_config: str = "",
    nav2_config: str = "",
    use_sim_time: str = "false",
    map_name: str = "",
    max_explore_radius: float = 0.0,
):
    if not slam_config or not nav2_config:
        raise ValueError(
            "both required: --slam_config <yaml> --nav2_config <yaml>"
        )

    print(f"[nav_experiment] slam_config = {slam_config}")
    print(f"[nav_experiment] nav2_config = {nav2_config}")
    print(f"[nav_experiment] map_name    = {map_name or '(none, explorer OFF)'}")

    bl = BetterLaunch()

    bl.include("slam_toolbox", "online_async_launch.py",
        **{"slam_params_file": slam_config, "use_sim_time": use_sim_time})

    # Trimmed nav2 launch (route_server/waypoint_follower/docking_server dropped)
    # to cut idle CPU on the Pi. See launch/nav2_experiment_navigation.launch.py.
    bl.include("dome_nav", "nav2_experiment_navigation.launch.py",
        **{"params_file": nav2_config, "use_sim_time": use_sim_time})

    # Explorer stack -- only when map_name given, so nav-only experiments still
    # run as before. Node params mirror robot_explore.launch.py (real-robot
    # explore settings) so behavior matches the core stack; only config yamls
    # differ per experiment. Core launch file is untouched.
    if map_name:
        home = dome_home()
        slam_map_path = os.path.join(home, "slam_maps", map_name)
        os.makedirs(home, exist_ok=True)

        bl.node(
            "dome_nav",
            "slam_manager_node",
            name="slam_manager",
            params={"map_persist_path": slam_map_path},
            ros_waittime=30.0,
            lifecycle_waittime=None,
        )

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
