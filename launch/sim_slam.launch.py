#!/usr/bin/env python3
# sim_slam.launch.py — slam_toolbox online_async, split out of sim_nav.launch.py
# (F13 T04) so it can be started and confirmed publishing /map before Nav2 is
# started separately via sim_nav2.launch.py. Requires sim_robot.launch.py
# already running for valid TF/scan/odom data. Map persistence (when running the
# full stack) is handled by slam_manager_node, so this file needs no map_name.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def sim_slam_launch():
    bl = BetterLaunch()

    pkg = get_package_share_directory("dome_nav")
    slam_config = os.path.join(pkg, "config", "mapper_params_online_async_sim.yaml")

    bl.include("slam_toolbox", "online_async_launch.py",
        **{"slam_params_file": slam_config, "use_sim_time": "true"})
