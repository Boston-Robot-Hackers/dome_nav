#!/usr/bin/env python3
# robot.launch.py — dome_nav on the physical robot: SLAM mapping (Mode A)
# Author: Pito Salas and Claude Code
# Version: 1
# Created: 2026-11-17
# Updated: 2026-11-17
# Open Source Under MIT license
#
# Usage:
#   bl dome_nav robot.launch.py --map_name <name>               # SLAM mapping
#   bl dome_nav robot.launch.py --mode slam --map_name <name>   # same, explicit
"""Start dome_nav's robot-side navigation stack for one mode.

The only mode is `slam`: slam_toolbox (via its upstream online_async launch)
plus slam_manager, which persists the map under $DOME_HOME/slam_maps/<name>.
Nav2 is not started. The Nav2 modes (`nav`, `explore`) return after dome_nav
I13, which reworks the real-robot params for Kilted.

Known problem: on Kilted, slam_toolbox sometimes stays `inactive` because its
configure reply is lost at startup (I13). Check with
`ros2 lifecycle get /slam_toolbox`.
"""

import os

from better_launch import BetterLaunch, launch_this

from dome_nav.utils import dome_home

MODES = ("slam",)


@launch_this(ui=True)
def robot_launch(mode: str = "slam", map_name: str = "", sim_time: bool = False):
    """Start dome_nav in the chosen mode.

    Parameters
    ----------
    mode : str
        Navigation mode. Only "slam" is available.
    map_name : str
        Required. Name of the map that slam_manager saves and reloads.
    sim_time : bool
        Use simulated time (/clock) instead of the wall clock.
    """
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}; valid modes: {', '.join(MODES)}")
    if not map_name:
        raise ValueError(
            "map_name is required: bl dome_nav robot.launch.py --map_name <name>"
        )

    bl = BetterLaunch()
    home = dome_home()
    os.makedirs(home, exist_ok=True)

    bl.include(
        "slam_toolbox",
        "online_async_launch.py",
        slam_params_file=bl.find("dome_nav", "mapper_params_online_async.yaml"),
        use_sim_time=str(sim_time).lower(),
    )
    bl.node(
        "dome_nav",
        "slam_manager_node",
        name="slam_manager",
        params={"map_persist_path": os.path.join(home, "slam_maps", map_name)},
        use_sim_time=sim_time,
        ros_waittime=30.0,
        lifecycle_waittime=None,
    )
