# F13 — Gazebo Simulation for Exploration Development

**Priority**: High
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Add a simulation launch mode that runs the full dome_nav exploration
stack (slam_toolbox + Nav2 + explore or pluggable explore node) inside Gazebo Harmonic
on a development machine, without physical hardware. Uses `ros_gz_sim` + `ros_gz_bridge`
(already installed with ROS2 Jazzy) with a self-contained SDF robot model.

## Scope

- `launch/sim_explore.launch.py` — single launch file that starts:
  - Gazebo Harmonic (`ros_gz_sim`) with `worlds/simple_room.world`
  - `ros_gz_bridge` for `/scan`, `/odom`, `/cmd_vel`, `/clock`, `/tf`
  - slam_toolbox online_async with `use_sim_time: true`
  - Nav2 stack with `use_sim_time: true`
  - `pluggable_explore_manager_node` with `use_sim_time: true`
  - All `use_sim_time` flags propagated consistently — sim time is mandatory
- `worlds/simple_room.world` — self-contained Gazebo Harmonic SDF: 8×8 m room,
  interior dividing wall with 1 m doorway, diff-drive robot with 2D lidar.
  Robot model defined in the world file; no external URDF or Fuel models required.
- Config: existing `*_param_patch.yaml` files reused; `use_sim_time` override
  added where not already present
- No new Python source files in `dome_nav/` — this feature is launch and config only

## Constraints

- `use_sim_time: true` must be set for every node; a node on wall clock while others
  run on sim time will produce incorrect TF and costmap behavior
- Gazebo Harmonic (`gz sim`) is what is installed on the dev machine; Gazebo Classic
  (`gazebo_ros_pkgs`) is not available on ROS2 Jazzy and must not be used
- `linorobot2_gazebo` cannot be used — it requires `gazebo_ros_pkgs` (Classic)
- Robot model is self-contained in the SDF world file; no external model URIs
- No changes to existing `robot_map.launch.py`, `robot_nav.launch.py`, or `robot_explore.launch.py`
- `map_name` arg is required (same convention as other launch files)

## How to Demo

**Setup**: ROS2 Jazzy environment sourced (`source /opt/ros/jazzy/setup.bash`).
No robot hardware required. No extra packages to install.

**Steps**:
1. `bl dome_nav sim_explore.launch.py --map_name sim_test`
2. Gazebo opens with robot spawned in simple_room world
3. RViz or Foxglove shows map growing as robot explores
4. Publish `exploration_start` intent: robot begins autonomous exploration
5. Robot visits all reachable frontier cells; map fills in
6. `exploration_stop` intent (or auto-stop on no frontiers) — robot halts, map saved

**Expected output**: complete occupancy grid of the simulated room with no
hardware. Exploration behavior is identical to Mode E on the real robot.
