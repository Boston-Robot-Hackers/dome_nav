#!/usr/bin/env python3
# robot_explore.launch.py — Mode A stack + frontier exploration for autonomous
# map building
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home


@launch_this(ui=True)
def robot_explore_launch(
    use_sim_time: str = "false",
    map_name: str = "",
    max_explore_radius: float = 0.0,
):
    if not map_name:
        raise ValueError(
            "map_name is required: bl robot_explore.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    home = dome_home()
    slam_map_path = os.path.join(home, "slam_maps", map_name)
    os.makedirs(home, exist_ok=True)

    pkg = get_package_share_directory("dome_nav")

    slam_config = os.path.join(pkg, "config", "slam_real.yaml")

    # Full standalone config, loaded verbatim -- no patch chain.
    nav2_config = os.path.join(pkg, "config", "nav2_explore_real.yaml")

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

    # Same explorer node as the sim stack (pluggable_explore_manager_node), differing
    # only by parameter values -- sim and real now share one code path. These are the
    # real-robot values (they match ExploreParams' own defaults except max_frontier_dist,
    # which the node declares as a sim-oriented 15.0 and is set back to 0.0 = unlimited
    # here). The sim launch files override min_frontier_dist/max_frontier_dist/
    # prefer_farthest/min_frontier_size for the simulated worlds.
    bl.node(
        "dome_nav",
        "pluggable_explore_manager_node",
        name="explore_manager",
        params={
            "max_explore_radius": max_explore_radius,
            "max_frontier_dist": 0.0,
            "min_frontier_dist": 1.3,
            "prefer_farthest": False,
            "min_frontier_size": 10,
            "map_name": map_name,
            "use_sim_time": use_sim_time == "true",
        },
        ros_waittime=30.0,
    )
