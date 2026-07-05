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
from dome_nav.utils import yaml_override, yaml_patch_dict


@launch_this(ui=True)
def sim_nav2_launch():
    bl = BetterLaunch()

    pkg = get_package_share_directory("dome_nav")
    nav2_base = bl.find("nav2_bringup", "nav2_params.yaml")
    nav2_patch = os.path.join(pkg, "config", "nav2_param_patch.yaml")
    explore_patch = os.path.join(pkg, "config", "explore_param_patch.yaml")
    dock_db = os.path.join(pkg, "config", "empty_dock_database.yaml")
    nav2_config = yaml_override(nav2_base, nav2_patch)
    nav2_config = yaml_override(nav2_config, explore_patch)
    nav2_config = yaml_patch_dict(nav2_config, {
        "docking_server": {"ros__parameters": {"dock_database": dock_db}}
    })

    # Sim-only speed bump — see sim_explore.launch.py for rationale.
    # Linear cruise speed raised 50% (0.3 -> 0.45) at user request; velocity_smoother's
    # linear cap raised to match (0.4 -> 0.6) so it doesn't clip the new desired speed.
    nav2_config = yaml_patch_dict(nav2_config, {
        "controller_server": {"ros__parameters": {
            "FollowPath": {"desired_linear_vel": 0.45},
        }},
        "velocity_smoother": {"ros__parameters": {
            "max_velocity": [0.6, 0.0, 1.9],
            "min_velocity": [-0.6, 0.0, -1.9],
            "max_accel": [1.5, 0.0, 3.2],
            "max_decel": [-1.5, 0.0, -3.2],
        }},
    })

    bl.include("nav2_bringup", "navigation_launch.py",
        **{"params_file": nav2_config, "use_sim_time": "true"})
