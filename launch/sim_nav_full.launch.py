#!/usr/bin/env python3
# sim_nav_full.launch.py — single-command full sim stack, composed from the
# existing single-purpose sim_*.launch.py files via bl.include() rather than
# duplicating their logic (as sim_explore.launch.py currently does). Includes,
# in the dependency order established during F13 T04 debugging: sim_robot
# (Gazebo/spawn/bridge/RSP/laser TF), sim_slam (must be up before Nav2 so the
# "map" TF frame exists), sim_nav2, then sim_explore_node. RViz is intentionally
# not included — sim_rviz.launch.py stays a separate, optional window.
# Blocks on wait_for_map_odom_tf() between the sim_slam and sim_nav2 includes:
# bl.include() only guarantees order, not readiness, and Nav2's global_costmap
# only waits 0.5s for the map->odom transform during activation before
# lifecycle_manager aborts the whole bringup (F13 T04t).
# Also starts slam_manager_node directly (not via an include, since none of
# the split files own it) so maps built through this single-command launch
# actually get persisted to ~/.dome/slam_maps/ like sim_explore.launch.py's do.
# Author: Pito Salas and Claude Code
# Open Source Under MIT license

import os
import time
from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this
from dome_nav.utils import dome_home, require_world_name
import rclpy
import tf2_ros


def wait_for_map_odom_tf(bl: BetterLaunch, timeout_s: float = 30.0) -> None:
    """Block until slam_toolbox's map->odom transform exists.

    bl.include() only guarantees launch order, not readiness -- slam_toolbox
    needs a moment after starting to receive its first /scan and publish this
    transform. Nav2's global_costmap only waits 0.5s for it during activation
    (hardcoded in nav2_costmap_2d, not YAML-configurable) and lifecycle_manager
    aborts the entire bringup if it times out, so launch order alone is not
    enough -- see 04-tasks/notdone/TF13-gazebo-simulation.md T04t.

    Uses bl.shared_node rather than a node of our own: better_launch runs
    rclpy.init() against its own private Context, not the global default one
    rclpy.create_node()/rclpy.init() implicitly target, so a plain
    rclpy.create_node() call here raises NotInitializedException.
    bl.shared_node is already spun continuously by better_launch's own
    background executor thread, so this only needs to poll the buffer, not
    call spin_once() itself -- doing so would race with that thread.
    """
    node = bl.shared_node
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    bl.logger.info(f"******* Waiting up to {timeout_s}s for map->odom transform...")
    start = time.time()
    while time.time() - start < timeout_s:
        if buffer.can_transform("map", "odom", rclpy.time.Time()):
            elapsed = time.time() - start
            bl.logger.info(f"*********** Map->odom transform available after {elapsed:.1f}s")
            return
        time.sleep(0.2)

    raise TimeoutError(
        f"******* map->odom transform did not appear within {timeout_s}s -- "
        "is slam_toolbox running and receiving /scan?"
    )


# Sim-only exploration defaults, kept identical across sim_explore.launch.py,
# sim_explore_node.launch.py, and sim_nav_full.launch.py. Can't be shared via an
# imported constant: bl's CLI statically parses launch function signatures via
# AST without importing the module (better_launch/utils/introspection.py), so a
# non-literal default like `= SOME_IMPORTED_NAME` fails with "not a valid float"
# -- only literal constants written directly in the signature work.
@launch_this(ui=True)
def sim_nav_full_launch(
    map_name: str = "",
    world_name: str = "",
    urdf_name: str = "minimal_sim.urdf",
    max_explore_radius: float = 0.0,
    max_frontier_dist: float = 15.0,
    min_frontier_dist: float = 0.9,
    preferred_goal_distance: float = 2.0,
    min_frontier_size: int = 5,
):
    if not map_name:
        raise ValueError(
            "map_name is required: bl dome_nav sim_nav_full.launch.py --map_name <name>"
        )
    require_world_name(
        world_name, os.path.join(get_package_share_directory("dome_nav"), "worlds"),
        "bl dome_nav sim_nav_full.launch.py --world_name <name>",
    )

    bl = BetterLaunch()

    bl.include("dome_nav", "sim_robot.launch.py")
    bl.include("dome_nav", "sim_slam.launch.py")
    wait_for_map_odom_tf(bl)
    bl.include("dome_nav", "sim_nav2.launch.py")
    bl.include("dome_nav", "sim_explore_node.launch.py")

    slam_map_path = os.path.join(dome_home(), "slam_maps", map_name)
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
