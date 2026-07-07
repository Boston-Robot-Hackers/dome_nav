#!/usr/bin/env python3
# sim_robot.launch.py — Combines sim_spawn, sim_bridge, sim_robot_state_publisher,
# and sim_laser_tf into one file: robot spawn, ros_gz_bridge, robot_state_publisher,
# and the static gz-laser-frame transform. Everything needed for a visible,
# TF-correct simulated robot, with no slam/Nav2/explore. Does NOT start Gazebo
# itself — start it yourself first:
#
#   gz sim -r <path-to-world>.world
#
# e.g. gz sim -r install/dome_nav/share/dome_nav/worlds/multi_room.world
#
# then run this file to spawn the robot into it. Run sim_nav.launch.py on top
# of this for navigation, or sim_rviz.launch.py to visualize.
#
# robot_state_publisher must be given an explicit name=. Without one, bl.node()
# treats it as anonymous and calls get_unique_name(), which scans every process
# on the system (get_nodes(include_foreign=True)) to avoid name collisions --
# on a busy VM this scan can take long enough that the node never appears to
# start at all (see 04-tasks/notdone/TF13-gazebo-simulation.md T04t).
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from better_launch import gazebo
from better_launch.gazebo import GazeboBridge
from dome_nav.utils import require_world_name, world_spawn_xy, write_config


@launch_this(ui=True)
def sim_robot_launch(world_name: str = ""):
    bl = BetterLaunch()

    pkg = get_package_share_directory("dome_nav")
    require_world_name(
        world_name, os.path.join(pkg, "worlds"),
        "bl dome_nav sim_robot.launch.py --world_name <name>",
    )
    urdf_path = os.path.join(pkg, "config", "dome3_sim.urdf")
    with open(urdf_path) as f:
        robot_description = f.read()

    spawn_x, spawn_y = world_spawn_xy(world_name)
    gazebo.spawn_model(
        "dome2",
        urdf_path,
        spawn_args=gazebo.get_gazebo_axes_args(x=spawn_x, y=spawn_y, z=0.05),
    )

    gazebo.spawn_topic_bridge(
        GazeboBridge.clock_bridge(),
        GazeboBridge("/scan", "sensor_msgs/msg/LaserScan", "gz2ros"),
        GazeboBridge("/odom", "nav_msgs/msg/Odometry", "gz2ros"),
        GazeboBridge("/tf", "tf2_msgs/msg/TFMessage", "gz2ros"),
        GazeboBridge("/cmd_vel", "geometry_msgs/msg/Twist", "ros2gz"),
        GazeboBridge("/model/dome2/joint_state", "sensor_msgs/msg/JointState", "gz2ros"),
    )

    # spawn_topic_bridge() always starts the bridge with raw=True, which drops any
    # remaps passed to it, so the bridge publishes under its literal topic name.
    # Remap robot_state_publisher's subscription instead, since bl.node() honors
    # remaps for non-raw nodes.
    #
    # robot_description is passed via a params file, not the params= dict: bl.node()
    # renders params= entries as individual "-p key:=<json value>" CLI args, and a
    # 300-line URDF blown up to one giant command-line argument hangs the process
    # spawn (same failure mode as the multi-line-XML CLI arg bug documented for
    # test5.bash). A params file avoids the command line entirely.
    rsp_params_path = write_config({
        "/**": {"ros__parameters": {
            "robot_description": robot_description,
            "use_sim_time": True,
        }}
    })
    bl.node(
        "robot_state_publisher",
        "robot_state_publisher",
        name="robot_state_publisher",
        param_files=[rsp_params_path],
        remaps={"/joint_states": "/model/dome2/joint_state"},
    )

    bl.node(
        "tf2_ros",
        "static_transform_publisher",
        name="gz_laser_frame_bridge",
        params={"use_sim_time": True},
        cmd_args=["0", "0", "0", "0", "0", "0", "laser", "dome2/base_footprint/lidar"],
    )
