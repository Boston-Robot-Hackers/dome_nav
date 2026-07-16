#!/usr/bin/env python3
# sim_explore.launch.py — Gazebo Harmonic simulation for autonomous exploration (F13)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from better_launch import gazebo
from better_launch.gazebo import GazeboBridge
from dome_nav.utils import (
    dome_home, require_world_name, world_spawn_xy, write_config,
)


# Sim-only exploration defaults, kept identical across sim_explore.launch.py,
# sim_explore_node.launch.py, and sim_nav_full.launch.py. Can't be shared via an
# imported constant: bl's CLI statically parses launch function signatures via
# AST without importing the module (better_launch/utils/introspection.py), so a
# non-literal default like `= SOME_IMPORTED_NAME` fails with "not a valid float"
# -- only literal constants written directly in the signature work.
@launch_this(ui=True)
def sim_explore_launch(
    map_name: str = "",
    world_name: str = "",
    max_explore_radius: float = 0.0,
    max_frontier_dist: float = 15.0,
    min_frontier_dist: float = 0.9,
    preferred_goal_distance: float = 2.0,
    min_frontier_size: int = 5,
):
    if not map_name:
        raise ValueError(
            "map_name is required: bl dome_nav sim_explore.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    home = dome_home()
    slam_map_path = os.path.join(home, "slam_maps", map_name)
    os.makedirs(os.path.join(home, "slam_maps"), exist_ok=True)

    pkg = get_package_share_directory("dome_nav")
    require_world_name(
        world_name, os.path.join(pkg, "worlds"),
        "bl dome_nav sim_explore.launch.py --world_name <name>",
    )
    urdf_path = os.path.join(pkg, "config", "dome3_sim.urdf")

    with open(urdf_path) as f:
        robot_description = f.read()

    slam_config = os.path.join(pkg, "config", "mapper_params_online_async_sim.yaml")

    # Full standalone config, loaded verbatim -- no patch chain.
    nav2_config = os.path.join(pkg, "config", "nav2_params_explore_sim.yaml")

    # Gazebo + robot spawn (GUI always on — needed to visually inspect costmap
    # inflation and robot behavior near obstacles during exploration debugging).
    spawn_x, spawn_y = world_spawn_xy(world_name)
    gazebo.gazebo_launch("dome_nav", f"{world_name}.world", gz_args=["-r"])
    gazebo.spawn_model(
        "dome2",
        urdf_path,
        spawn_args=gazebo.get_gazebo_axes_args(x=spawn_x, y=spawn_y, z=0.05),
    )

    # ros_gz_bridge — all topics needed by slam_toolbox, Nav2, and explore node
    gazebo.spawn_topic_bridge(
        GazeboBridge.clock_bridge(),
        GazeboBridge("/scan", "sensor_msgs/msg/LaserScan", "gz2ros"),
        GazeboBridge("/odom", "nav_msgs/msg/Odometry", "gz2ros"),
        GazeboBridge("/tf", "tf2_msgs/msg/TFMessage", "gz2ros"),
        GazeboBridge("/cmd_vel", "geometry_msgs/msg/Twist", "ros2gz"),
        GazeboBridge(
            "/model/dome2/joint_state", "sensor_msgs/msg/JointState", "gz2ros"
        ),
    )

    # robot_state_publisher — fixed-joint TF (base_footprint→base_link→laser etc.)
    # spawn_topic_bridge() always starts the bridge with raw=True, which drops any
    # remaps passed to it, so the bridge publishes under its literal topic name.
    # Remap robot_state_publisher's subscription instead, since bl.node() honors
    # remaps for non-raw nodes.
    #
    # robot_state_publisher must be given an explicit name=, and robot_description
    # must go through a params file rather than params=: see F13 T04t
    # (04-tasks/notdone/TF13-gazebo-simulation.md) for why an anonymous name here
    # hangs the launch indefinitely on a busy VM.
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

    # gz-sim renames the lidar sensor to "dome2/base_footprint/lidar" after fixed-joint
    # reduction. This static TF anchors that gz frame to the URDF "laser" frame so
    # slam_toolbox can look up the scan's frame_id in the TF tree.
    bl.node(
        "tf2_ros",
        "static_transform_publisher",
        name="gz_laser_frame_bridge",
        params={"use_sim_time": True},
        cmd_args=["0", "0", "0", "0", "0", "0", "laser", "dome2/base_footprint/lidar"],
    )

    # slam_toolbox
    bl.include("slam_toolbox", "online_async_launch.py",
        **{"slam_params_file": slam_config, "use_sim_time": "true"})

    # Nav2
    bl.include("nav2_bringup", "navigation_launch.py",
        **{"params_file": nav2_config, "use_sim_time": "true"})

    # slam_manager_node
    bl.node(
        "dome_nav",
        "slam_manager_node",
        name="slam_manager",
        params={
            "map_persist_path": slam_map_path,
            "use_sim_time": True,
            "save_period_sec": 60.0,
        },
        ros_waittime=30.0,
        lifecycle_waittime=None,
    )

    # explorer_manager_node
    bl.node(
        "dome_nav",
        "explorer_manager_node",
        name="explore_manager",
        params={
            "max_explore_radius": max_explore_radius,
            "max_frontier_dist": max_frontier_dist,
            "min_frontier_dist": min_frontier_dist,
            "preferred_goal_distance": preferred_goal_distance,
            "min_frontier_size": min_frontier_size,
            "map_name": map_name,
            "use_sim_time": True,
        },
        ros_waittime=30.0,
    )
