#!/usr/bin/env python3
# sim_nav_default.launch.py — EXPERIMENTAL / DIAGNOSTIC (2026-07-09).
#
# Same sim + robot + Nav2 stack as sim_nav_full.launch.py, but Nav2 is loaded
# with nav2_bringup's OWN stock params (nav2_params.yaml) instead of this
# project's config/nav2_explore_sim.yaml. Purpose: bisect the "robot receives
# paths for 25s but only moves 3cm" bug. If the robot drives to a goal under
# the stock config, the fault is in nav2_explore_sim.yaml (MPPI tune,
# velocity_smoother, or collision_monitor wiring). If it still won't move, the
# fault is in the sim/robot wiring (the /cmd_vel bridge, TF, or odom).
#
# No exploration node and no slam_manager here — this is a bench test. Send a
# goal by hand once the stack is up:
#   * RViz:  bl dome_nav sim_rviz.launch.py  -> "2D Goal Pose"
#   * CLI:   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
#              "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, \
#               orientation: {w: 1.0}}}}"
#
# Then watch what actually reaches Gazebo:
#   ros2 topic echo /cmd_vel            # bridged into Gazebo (ros2gz)
#   ros2 topic echo /cmd_vel_nav        # stock nav2 controller output (pre-smoother)
#
# Not wired into any of the working launch files; delete once the bug is found.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
import time
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import require_world_name
import rclpy
import tf2_ros


def wait_for_map_odom_tf(bl: BetterLaunch, timeout_s: float = 30.0) -> None:
    """Block until slam_toolbox's map->odom transform exists.

    Copied from sim_nav_full.launch.py (kept local so this experimental file
    depends on nothing importable): bl.include() guarantees launch order, not
    readiness, and Nav2's global_costmap only waits 0.5s for map->odom during
    activation before lifecycle_manager aborts the whole bringup.
    """
    node = bl.shared_node
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    bl.logger.info(f"******* Waiting up to {timeout_s}s for map->odom transform...")
    start = time.time()
    while time.time() - start < timeout_s:
        if buffer.can_transform("map", "odom", rclpy.time.Time()):
            elapsed = time.time() - start
            bl.logger.info(f"map->odom transform available after {elapsed:.1f}s")
            return
        time.sleep(0.2)

    raise TimeoutError(
        f"******* map->odom transform did not appear within {timeout_s}s -- "
        "is slam_toolbox running and receiving /scan?"
    )


@launch_this(ui=True)
def sim_nav_default_launch(
    world_name: str = "",
    urdf_name: str = "dome3_sim.urdf",
):
    require_world_name(
        world_name, os.path.join(get_package_share_directory("dome_nav"), "worlds"),
        "bl dome_nav sim_nav_default.launch.py --world_name <name>",
    )

    bl = BetterLaunch()

    # Robot + Gazebo + bridge + RSP + laser TF, then slam for the map frame.
    bl.include("dome_nav", "sim_robot.launch.py")
    bl.include("dome_nav", "sim_slam.launch.py")
    wait_for_map_odom_tf(bl)

    # Nav2 with its stock params instead of config/nav2_explore_sim.yaml.
    nav2_default = os.path.join(
        get_package_share_directory("nav2_bringup"), "params", "nav2_params.yaml"
    )
    bl.include("nav2_bringup", "navigation_launch.py",
        **{"params_file": nav2_default, "use_sim_time": "true"})
