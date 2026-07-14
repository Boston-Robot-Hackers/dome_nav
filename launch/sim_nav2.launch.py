#!/usr/bin/env python3
# sim_nav2.launch.py — Nav2 stack, split out of sim_nav.launch.py (F13 T04) so
# it can be started separately from slam_toolbox, once slam is confirmed
# publishing /map. Requires sim_robot.launch.py and sim_slam.launch.py already
# running — without a "map" frame, planner_server's global_costmap blocks on
# activation and lifecycle_manager aborts the entire bringup after ~60s.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def sim_nav2_launch():
    bl = BetterLaunch()

    pkg = get_package_share_directory("dome_nav")
    # Full standalone config, loaded verbatim -- no patch chain.
    nav2_config = os.path.join(pkg, "config", "nav2_params_explore_sim.yaml")

    bl.include("nav2_bringup", "navigation_launch.py",
        **{"params_file": nav2_config, "use_sim_time": "true"})
