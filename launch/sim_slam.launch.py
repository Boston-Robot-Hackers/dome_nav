#!/usr/bin/env python3
# sim_slam.launch.py — slam_toolbox online_async, split out of sim_nav.launch.py
# (F13 T04) so it can be started and confirmed publishing /map before Nav2 is
# started separately via sim_nav2.launch.py. Requires sim_robot.launch.py
# already running for valid TF/scan/odom data.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home, yaml_override, yaml_patch_dict


@launch_this(ui=True)
def sim_slam_launch(map_name: str = ""):
    if not map_name:
        raise ValueError(
            "map_name is required: bl dome_nav sim_slam.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    home = dome_home()
    slam_map_path = os.path.join(home, "slam_maps", map_name)
    os.makedirs(os.path.join(home, "slam_maps"), exist_ok=True)

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
