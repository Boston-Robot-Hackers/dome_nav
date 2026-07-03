#!/usr/bin/env python3
# sim_robot.launch.py — Combines sim_gazebo, sim_spawn, sim_bridge,
# sim_robot_state_publisher, and sim_laser_tf into one file: Gazebo GUI, robot
# spawn, ros_gz_bridge, robot_state_publisher, and the static gz-laser-frame
# transform. Everything needed for a visible, TF-correct simulated robot, with
# no slam/Nav2/explore. Run sim_nav.launch.py on top of this for navigation, or
# sim_rviz.launch.py to visualize.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from better_launch import gazebo
from better_launch.gazebo import GazeboBridge


@launch_this(ui=True)
def sim_robot_launch():
    bl = BetterLaunch()

    pkg = get_package_share_directory("dome_nav")
    urdf_path = os.path.join(pkg, "config", "dome3_sim.urdf")
    with open(urdf_path) as f:
        robot_description = f.read()

    gazebo.gazebo_launch("dome_nav", "simple_room.world", gz_args=["-r"])
    gazebo.spawn_model(
        "dome2",
        urdf_path,
        spawn_args=gazebo.get_gazebo_axes_args(x=-1.0, y=-1.0, z=0.05),
    )

    gazebo.spawn_topic_bridge(
        GazeboBridge.clock_bridge(),
        GazeboBridge("/scan", "sensor_msgs/msg/LaserScan", "gz2ros"),
        GazeboBridge("/odom", "nav_msgs/msg/Odometry", "gz2ros"),
        GazeboBridge("/tf", "tf2_msgs/msg/TFMessage", "gz2ros"),
        GazeboBridge("/cmd_vel", "geometry_msgs/msg/Twist", "ros2gz"),
        GazeboBridge("/model/dome2/joint_state", "sensor_msgs/msg/JointState", "gz2ros",
                     remaps={"/model/dome2/joint_state": "/joint_states"}),
    )

    bl.node(
        "robot_state_publisher",
        "robot_state_publisher",
        params={"robot_description": robot_description, "use_sim_time": True},
    )

    bl.node(
        "tf2_ros",
        "static_transform_publisher",
        name="gz_laser_frame_bridge",
        params={"use_sim_time": True},
        cmd_args=["0", "0", "0", "0", "0", "0", "laser", "dome2/base_footprint/lidar"],
    )
