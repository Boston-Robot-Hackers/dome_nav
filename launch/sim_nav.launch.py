#!/usr/bin/env python3
# sim_nav.launch.py — Combines sim_slam and sim_nav2 into one file, in the
# required order: slam_toolbox must be up and publishing the "map" frame before
# Nav2's planner_server can activate (see TF13 T04 for the live-debugging finding
# that motivated this — Nav2 alone times out waiting on base_link->map TF and
# lifecycle_manager aborts the whole bringup). Requires sim_robot.launch.py
# already running for valid TF/scan/odom data.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home, yaml_override, yaml_patch_dict


@launch_this(ui=True)
def sim_nav_launch(map_name: str = ""):
    if not map_name:
        raise ValueError(
            "map_name is required: bl dome_nav sim_nav.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    home = dome_home()
    slam_map_path = os.path.join(home, "slam_maps", map_name)
    os.makedirs(slam_map_path, exist_ok=True)

    pkg = get_package_share_directory("dome_nav")
    slam_base = bl.find("slam_toolbox", "mapper_params_online_async.yaml")
    slam_patch = os.path.join(pkg, "config", "slam_param_patch.yaml")
    slam_config = yaml_override(slam_base, slam_patch)
    slam_config = yaml_patch_dict(slam_config, {
        "slam_toolbox": {"ros__parameters": {
            "map_file_name": slam_map_path,
            "use_sim_time": True,
        }}
    })

    bl.include("slam_toolbox", "online_async_launch.py",
        **{"slam_params_file": slam_config, "use_sim_time": "true"})

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
    nav2_config = yaml_patch_dict(nav2_config, {
        "controller_server": {"ros__parameters": {
            "FollowPath": {"desired_linear_vel": 0.3},
        }},
        "velocity_smoother": {"ros__parameters": {
            "max_velocity": [0.4, 0.0, 1.9],
            "min_velocity": [-0.4, 0.0, -1.9],
            "max_accel": [1.5, 0.0, 3.2],
            "max_decel": [-1.5, 0.0, -3.2],
        }},
    })

    bl.include("nav2_bringup", "navigation_launch.py",
        **{"params_file": nav2_config, "use_sim_time": "true"})
