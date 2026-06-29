# TF13 — Gazebo Simulation for F13

## T01 — Verify linorobot2_gazebo launches standalone
**Status**: not done
**Description**: Run `linorobot2_gazebo gazebo.launch.py` on the dev machine and confirm
Gazebo opens, the robot spawns, `/scan` and `/odom` topics publish. Record any missing
packages or build steps needed.
**Test**: Manual — confirm `/scan` and `/odom` echo within 10 s of launch.

## T02 — Create simple_room.world
**Status**: not done
**Description**: Write `worlds/simple_room.world` — a Gazebo Classic SDF world with a
single enclosed room (~8×8 m), one interior wall with a doorway, and no external
dependencies. Robot spawn point clear of all walls. Sized so frontier exploration
visits the whole space in under 2 minutes.
**Test**: Gazebo loads the world without errors; robot spawns without clipping geometry.

## T03 — Create sim_explore.launch.py
**Status**: not done
**Description**: New file `launch/sim_explore.launch.py` using `better_launch` conventions.
Requires `map_name` arg (error if missing). Includes in order:
linorobot2 Gazebo spawn (world defaulting to `simple_room.world`),
slam_toolbox online_async (`use_sim_time: true`),
Nav2 stack (`use_sim_time: true`, reusing `nav2_param_patch.yaml`),
`pluggable_explore_manager_node` (`use_sim_time: true`, reusing `explore_param_patch.yaml`).
All nodes receive `use_sim_time: true`.
**Test**: `bl dome_nav sim_explore.launch.py --map_name sim_test` launches without
errors; `/map`, `/scan`, `/odom`, `/explore/status` all present within 15 s.

## T04 — Verify sim_time propagation
**Status**: not done
**Description**: With sim_explore.launch.py running, confirm that TF timestamps,
costmap stamps, and Nav2 action timestamps are all on sim clock. Check with
`ros2 topic echo /clock` and `ros2 topic echo /tf --once`. A wall-clock node will
show stale or zero-latency stamps — fix any that appear.
**Test**: Manual — no TF warnings about old timestamps in the node logs; costmap
updates visible in RViz synchronized with sim time.

## T05 — End-to-end exploration smoke test
**Status**: not done
**Description**: Full demo of the feature as described in F13 How to Demo:
launch sim, publish `exploration_start`, observe robot driving and map filling,
publish `exploration_stop` or wait for auto-stop, confirm map saved to
`~/.dome/slam_maps/sim_test/`.
**Test**: Manual — record `/explore/status` transitions seen, confirm map file written,
note any navigation failures or stuck behavior.

## T06 — Update feature file and current.md
**Status**: not done
**Description**: Set `Tasks File Created: yes` in `F13-gazebo-simulation.md`.
After T05 passes, set `Done: yes`, `Tests Written: yes`, `Test Passing: yes`,
move feature to `03-features/done/` and task file to `04-tasks/done/`.
Update `02-doc/current.md` with F13 summary.
**Test**: Not applicable — housekeeping only.
