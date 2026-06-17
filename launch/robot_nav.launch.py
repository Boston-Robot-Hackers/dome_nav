#!/usr/bin/env python3
# robot_nav.launch.py — Mode B: static map + AMCL + Nav2 for normal robot operation.
# Requires a saved map at ~/.dome/slam_maps/basement1.yaml (built with robot_map.launch.py).
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home, yaml_override, yaml_patch_dict


@launch_this(ui=True)
def robot_nav_launch(use_sim_time: str = "false"):
    bl = BetterLaunch()

    home = dome_home()
    map_path = os.path.join(home, "slam_maps", "basement1.yaml")

    pkg = get_package_share_directory("dome_nav")
    amcl_patch = os.path.join(pkg, "config", "nav2_amcl_patch.yaml")
    nav2_patch = os.path.join(pkg, "config", "nav2_param_patch.yaml")

    # Localization: map_server + AMCL (provides map→odom TF, replaces slam_toolbox)
    nav2_base = bl.find("nav2_bringup", "nav2_params.yaml")
    loc_config = yaml_override(nav2_base, amcl_patch)
    loc_config = yaml_patch_dict(loc_config, {
        "map_server": {"ros__parameters": {"yaml_filename": map_path}}
    })

    bl.include("nav2_bringup", "localization_launch.py",
        map=map_path, params_file=loc_config, use_sim_time=use_sim_time)

    # Navigation: planner + controller + costmap (no AMCL, no map_server)
    nav_config = yaml_override(nav2_base, nav2_patch)

    bl.include("nav2_bringup", "navigation_launch.py",
        params_file=nav_config, use_sim_time=use_sim_time)

    bl.node(
        "dome_nav",
        "nav_manager_node",
        name="nav_manager",
        ros_waittime=30.0,
    )
