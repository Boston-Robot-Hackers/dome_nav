#!/usr/bin/env python3
# robot_map.launch.py — slam_toolbox + Nav2 + dome_nav nodes for the physical robot
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home


@launch_this(ui=True)
def robot_launch(use_sim_time: str = "false", map_name: str = ""):
    if not map_name:
        raise ValueError(
            "map_name is required: bl robot_map.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    home = dome_home()
    slam_map_path = os.path.join(home, "slam_maps", map_name)
    os.makedirs(home, exist_ok=True)

    pkg = get_package_share_directory("dome_nav")

    slam_config = os.path.join(pkg, "config", "mapper_params_online_async.yaml")
    nav2_config = os.path.join(pkg, "config", "nav2_params_real.yaml")

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

    bl.node(
        "dome_nav",
        "nav_manager_node",
        name="nav_manager",
        ros_waittime=30.0,
    )
