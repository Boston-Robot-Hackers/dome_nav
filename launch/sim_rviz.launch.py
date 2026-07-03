#!/usr/bin/env python3
# sim_rviz.launch.py — Starts RViz2 with use_sim_time on. In RViz2: set Fixed
# Frame to "odom", then add displays for RobotModel, LaserScan (topic /scan),
# and TF.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def sim_rviz_launch():
    bl = BetterLaunch()

    bl.node("rviz2", "rviz2", params={"use_sim_time": True})
