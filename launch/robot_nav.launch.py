#!/usr/bin/env python3
# robot_nav.launch.py — Mode B: static map + AMCL + Nav2 for normal robot operation.
# Requires a saved map at ~/.dome/slam_maps/basement1.yaml (built with
# robot_map.launch.py).
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home


@launch_this(ui=True)
def robot_nav_launch(use_sim_time: str = "false"):
    bl = BetterLaunch()

    home = dome_home()
    map_path = os.path.join(home, "slam_maps", "basement1.yaml")

    pkg = get_package_share_directory("dome_nav")

    # Localization: map_server + AMCL (provides map→odom TF, replaces slam_toolbox).
    # localization_launch.py sets map_server's yaml_filename from its own map= arg,
    # so the per-map path stays a launch arg -- not baked into the config.
    loc_config = os.path.join(pkg, "config", "nav2_params_localization_real.yaml")

    bl.include("nav2_bringup", "localization_launch.py",
        map=map_path, params_file=loc_config, use_sim_time=use_sim_time)

    # Navigation: planner + controller + costmap (no AMCL, no map_server)
    nav_config = os.path.join(pkg, "config", "nav2_params_real.yaml")

    bl.include("nav2_bringup", "navigation_launch.py",
        params_file=nav_config, use_sim_time=use_sim_time,
        use_docking_server="False")

    bl.node(
        "dome_nav",
        "nav_manager_node",
        name="nav_manager",
        ros_waittime=30.0,
    )
