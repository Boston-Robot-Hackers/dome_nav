#!/usr/bin/env python3
# robot.launch.py — slam_toolbox + Nav2 + dome_nav nodes for the physical robot
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home, yaml_override, yaml_patch_dict


@launch_this(ui=True)
def robot_launch(use_sim_time: str = "false", map_name: str = "basement1"):
    bl = BetterLaunch()

    home = dome_home()
    slam_map_path = os.path.join(home, "slam_maps", map_name)
    os.makedirs(home, exist_ok=True)

    pkg = get_package_share_directory("dome_nav")

    slam_base = bl.find("slam_toolbox", "mapper_params_online_async.yaml")
    slam_patch = os.path.join(pkg, "config", "slam_param_patch.yaml")
    slam_config = yaml_override(slam_base, slam_patch)
    slam_config = yaml_patch_dict(slam_config, {
        "slam_toolbox": {"ros__parameters": {"map_file_name": slam_map_path}}
    })

    nav2_base = bl.find("nav2_bringup", "nav2_params.yaml")
    nav2_patch = os.path.join(pkg, "config", "nav2_param_patch.yaml")
    nav2_config = yaml_override(nav2_base, nav2_patch)

    bl.include("slam_toolbox", "online_async_launch.py",
        **{"slam_params_file": slam_config})

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
