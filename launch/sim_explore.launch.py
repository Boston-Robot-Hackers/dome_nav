#!/usr/bin/env python3
# sim_explore.launch.py — Gazebo Harmonic simulation for autonomous exploration (F13)
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from better_launch import gazebo
from better_launch.gazebo import GazeboBridge
from dome_nav.utils import dome_home, yaml_override, yaml_patch_dict


@launch_this(ui=True)
def sim_explore_launch(
    map_name: str = "",
    max_explore_radius: float = 0.0,
    max_frontier_dist: float = 1.0,
):
    if not map_name:
        raise ValueError(
            "map_name is required: bl dome_nav sim_explore.launch.py --map_name <name>"
        )

    bl = BetterLaunch()

    home = dome_home()
    slam_map_path = os.path.join(home, "slam_maps", map_name)
    os.makedirs(slam_map_path, exist_ok=True)

    pkg = get_package_share_directory("dome_nav")
    urdf_path = os.path.join(pkg, "config", "dome3_sim.urdf")

    with open(urdf_path) as f:
        robot_description = f.read()

    slam_base = bl.find("slam_toolbox", "mapper_params_online_async.yaml")
    slam_patch = os.path.join(pkg, "config", "slam_param_patch.yaml")
    slam_config = yaml_override(slam_base, slam_patch)
    slam_config = yaml_patch_dict(slam_config, {
        "slam_toolbox": {"ros__parameters": {
            "map_file_name": slam_map_path,
            "use_sim_time": True,
        }}
    })

    nav2_base = bl.find("nav2_bringup", "nav2_params.yaml")
    nav2_patch = os.path.join(pkg, "config", "nav2_param_patch.yaml")
    explore_patch = os.path.join(pkg, "config", "explore_param_patch.yaml")
    dock_db = os.path.join(pkg, "config", "empty_dock_database.yaml")
    nav2_config = yaml_override(nav2_base, nav2_patch)
    nav2_config = yaml_override(nav2_config, explore_patch)
    nav2_config = yaml_patch_dict(nav2_config, {
        "docking_server": {"ros__parameters": {"dock_database": dock_db}}
    })

    # Sim-only speed bump: explore_param_patch.yaml intentionally caps exploration
    # speed low to protect slam_toolbox scan-matching on real hardware. That
    # constraint doesn't apply to development/testing in Gazebo, so raise the
    # cap here rather than touching the shared real-robot config files.
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

    # Gazebo + robot spawn (GUI always on — needed to visually inspect costmap
    # inflation and robot behavior near obstacles during exploration debugging).
    gazebo.gazebo_launch("dome_nav", "simple_room.world", gz_args=["-r"])
    gazebo.spawn_model(
        "dome2",
        urdf_path,
        spawn_args=gazebo.get_gazebo_axes_args(x=-1.0, y=-1.0, z=0.05),
    )

    # ros_gz_bridge — all topics needed by slam_toolbox, Nav2, and explore node
    gazebo.spawn_topic_bridge(
        GazeboBridge.clock_bridge(),
        GazeboBridge("/scan", "sensor_msgs/msg/LaserScan", "gz2ros"),
        GazeboBridge("/odom", "nav_msgs/msg/Odometry", "gz2ros"),
        GazeboBridge("/tf", "tf2_msgs/msg/TFMessage", "gz2ros"),
        GazeboBridge("/cmd_vel", "geometry_msgs/msg/Twist", "ros2gz"),
        GazeboBridge("/model/dome2/joint_state", "sensor_msgs/msg/JointState", "gz2ros",
                     remaps={"/model/dome2/joint_state": "/joint_states"}),
    )

    # robot_state_publisher — fixed-joint TF (base_footprint→base_link→laser etc.)
    bl.node(
        "robot_state_publisher",
        "robot_state_publisher",
        params={"robot_description": robot_description, "use_sim_time": True},
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
            "save_period_sec": 120.0,
        },
        ros_waittime=30.0,
        lifecycle_waittime=None,
    )

    # pluggable_explore_manager_node
    bl.node(
        "dome_nav",
        "pluggable_explore_manager_node",
        name="explore_manager",
        params={
            "max_explore_radius": max_explore_radius,
            "max_frontier_dist": max_frontier_dist,
            "map_name": map_name,
            "use_sim_time": True,
        },
        ros_waittime=30.0,
    )
