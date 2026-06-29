# F13 — Gazebo Simulation for Exploration Development

**Priority**: High
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: Add a simulation launch mode that runs the full dome_nav exploration
stack (slam_toolbox + Nav2 + explore or pluggable explore node) inside Gazebo on a
development machine, without physical hardware. Uses `linorobot2_gazebo` (already in
the workspace) as the robot and sensor simulator.

## Scope

- `launch/sim_explore.launch.py` — single launch file that starts:
  - Gazebo with a chosen world (`playground`, `gas_station`, or a new simple room world)
  - linorobot2 robot spawn (URDF via `linorobot2_description`)
  - slam_toolbox online_async with `use_sim_time: true`
  - Nav2 stack with `use_sim_time: true`
  - `pluggable_explore_manager_node` or `explore_manager_node`
  - All `use_sim_time` flags propagated consistently — sim time is mandatory
- A minimal Gazebo world (`worlds/simple_room.world`) sized for indoor exploration:
  rooms, corridors, doorways that test frontier detection and navigation
- Config: all existing `*_param_patch.yaml` files reused; `use_sim_time` override
  added where not already present
- No new Python source files in `dome_nav/` — this feature is launch and config only

## Constraints

- `use_sim_time: true` must be set for every node; a node that runs on wall clock
  while others run on sim time will produce incorrect TF and costmap behavior
- `linorobot2_gazebo` is already in the workspace — do not re-implement robot spawning
- Gazebo Classic (gazebo_ros) is what linorobot2_gazebo uses; do not switch to gz-sim/Harmonic
- No changes to existing `robot_map.launch.py`, `robot_nav.launch.py`, or `robot_explore.launch.py`
- `map_name` arg is required (same convention as other launch files)

## How to Demo

**Setup**: ROS2 Jazzy environment sourced; workspace built; Gazebo Classic installed
(`ros-jazzy-gazebo-ros-pkgs` or equivalent). No robot hardware required.

**Steps**:
1. `bl dome_nav sim_explore.launch.py --map_name sim_test`
2. Gazebo opens with robot spawned in simple_room world
3. RViz or Foxglove shows map growing as robot explores
4. Publish `exploration_start` intent: robot begins autonomous exploration
5. Robot visits all reachable frontier cells; map fills in
6. `exploration_stop` intent (or auto-stop on no frontiers) — robot halts, map saved

**Expected output**: complete occupancy grid of the simulated room with no
hardware. Exploration behavior is identical to Mode E on the real robot.
