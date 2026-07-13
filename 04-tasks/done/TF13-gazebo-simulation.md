# TF13 — Gazebo Simulation for F13

## T01 — Verify Gazebo availability and choose simulator path
**Status**: done
**Decision**: Use Gazebo Harmonic (`gz sim`) with `ros_gz_bridge`. Do NOT use `linorobot2_gazebo` (requires Classic). `gz sim 8.11.0` confirmed installed via `gz_sim_vendor`.

## T02a — Create simple_room.world
**Status**: done
**Description**: `worlds/simple_room.world` — 8×8 m room, interior dividing wall with 1 m doorway. Gazebo Harmonic SDF, no external model URIs.

## T02b — Create dome3_sim.urdf
**Status**: done
**Description**: `config/dome3_sim.urdf` — diff-drive robot with inertia, friction, gpu_lidar sensor, JointStatePublisher plugin. Note: sensor must be `type="gpu_lidar"` not `type="lidar"` — the plain lidar type is not driven by the Sensors system and produces no output.

## T03 — Create sim_explore.launch.py
**Status**: done
**Description**: `launch/sim_explore.launch.py` exists. Superseded as primary launch by `sim_nav_full.launch.py` (T04b), but retained as an alternative single-file launch.

## T04 — Sim stack integration, debugging, and tuning
**Status**: done
**Summary of what was built and why:**
- `sim_nav_full.launch.py` — single-command full stack (Gazebo + spawn + bridge + RSP + laser TF + slam + Nav2 + explore node + slam_manager). Gazebo launched inside `sim_robot.launch.py` via `gazebo.gazebo_launch()`.
- `sim_robot.launch.py`, `sim_slam.launch.py`, `sim_nav2.launch.py`, `sim_explore_node.launch.py`, `sim_rviz.launch.py` — split single-job files for step-by-step debugging.
- `worlds/multi_room.world` — 10×10 m multi-room floorplan with 2 m doorways; robot spawns at (1,1). `simple_room.world`'s 0.6 m doorway was too narrow for inflation + robot radius.
- `world_name` launch arg required; `world_spawn_xy()` maps world to spawn point.
- `config/minimal_sim.urdf` — minimal URDF used by default (dome3_sim.urdf retained as alternate).
- Standalone config files (`slam_sim.yaml`, `nav2_explore_sim.yaml`) replace all runtime YAML patching.
- `robot_state_publisher` must have `name=` explicit — anonymous naming triggers full-system process scan that stalls on busy VMs.
- `wait_for_map_odom_tf()` blocks between slam and Nav2 includes — Nav2's global_costmap only waits 0.5 s for `map→odom` TF during activation.
- Frontier params (sim defaults): `min_frontier_dist` 0.9 m, `max_frontier_dist` 15.0 m, `min_frontier_size` 5, `prefer_farthest` True.
- Buffer-cell frontier definition: frontier cell must not itself touch unknown — must have a 4-neighbor that does. Keeps goals one cell off the known/unknown boundary.
- Mid-navigation redirect disabled (`check_goal_redirect` removed from `explore_tick`).
- slam_toolbox sim overrides in `slam_sim.yaml`: `minimum_travel_distance`/`minimum_travel_heading` 0.1, `map_update_interval` 1.0.
- MPPI `batch_size` and `controller_frequency` tuned in `nav2_explore_sim.yaml`.
- **Confirmed working**: robot drives and covers multi_room.world (~16 goals over ~9×9 m area in one run).

## T04a — Fix stray empty map-name directory
**Status**: done — `sim_slam.launch.py` and `sim_explore.launch.py` now only `makedirs` the parent `slam_maps/` dir, not `slam_maps/<map_name>/`.

## T04t — Fix robot_state_publisher never starting
**Status**: done — must pass `name="robot_state_publisher"` explicitly to `bl.node()`. Also fixed `better_launch/elements/node.py`: `--param-file` → `--params-file`.

## T04x — Remove all YAML patching
**Status**: done — standalone config files in `config/`: `slam_real.yaml`, `slam_sim.yaml`, `nav2_real.yaml`, `nav2_localization_real.yaml`, `nav2_explore_real.yaml`, `nav2_explore_sim.yaml`. No merge/patch at runtime.

## T05 — End-to-end exploration smoke test
**Status**: done — robot confirmed driving and covering multi_room.world (~16 goals, ~9×9 m area). See `02-doc/current.md`.

## T06 — Update feature file and current.md
**Status**: not done
**Description**: Set `Done: yes`, `Tests Written: yes`, `Test Passing: yes` in `F13-gazebo-simulation.md`. Move feature to `03-features/done/` and this task file to `04-tasks/done/`. Update `02-doc/current.md`.
