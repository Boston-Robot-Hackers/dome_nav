# TF13 — Gazebo Simulation for F13

## T01 — Verify Gazebo availability and choose simulator path
**Status**: done
**Description**: Investigated simulator availability on the dev machine.
**Findings**:
- Gazebo Classic (`gazebo`, `gazebo_ros_pkgs`) is NOT installed and NOT available for ROS2 Jazzy.
- Gazebo Harmonic (gz sim 8.11.0) IS installed via `gz_sim_vendor`; bridge packages
  `ros_gz_bridge`, `ros_gz_sim`, `ros_gz_interfaces` are all present.
- `linorobot2_gazebo` depends on `gazebo_ros_pkgs` (Classic) — cannot build or run.
- `nav2_minimal_tb3_sim` is installed (Nav2's built-in Gazebo Harmonic turtlebot3 sim).
**Decision**: Use Gazebo Harmonic (`gz sim`) with `ros_gz_bridge`. Use `nav2_minimal_tb3_sim`
or a hand-crafted SDF world. Do NOT use `linorobot2_gazebo`. Update F13 feature file to
remove the "use Gazebo Classic" constraint and replace with "use Gazebo Harmonic".
**Test**: `gz sim --version` → `Gazebo Sim, version 8.11.0`. Confirmed.

## T02a — Create simple_room.world (room geometry only)
**Status**: done
**Description**: Write `worlds/simple_room.world` as a Gazebo Harmonic SDF world.
Room geometry only — no embedded robot model. Contains: ground plane, sun, four outer
walls (8×8 m, 0.15 m thick, 1.5 m tall), interior dividing wall with 1 m doorway.
World-level plugins: Physics, Sensors (ogre2), SceneBroadcaster, UserCommands.
No external model URIs. Robot is spawned separately via ros_gz_sim + robot_description.
**Test**: `gz sim -s worlds/simple_room.world --iterations 50` exits cleanly; no errors.

## T02b — Create dome3_sim.urdf
**Status**: done
**Description**: New file `config/dome3_sim.urdf` based on `dome2/config/dome3.urdf`.
Adds simulation requirements without touching the original:
- Inertia on base_link (cylinder r=0.15, l=0.003, m=2.5 kg), left_wheel and
  right_wheel (cylinder r=0.03, l=0.0175, m=0.2 kg), caster (sphere r=0.015, m=0.05 kg)
- Friction on left_wheel and right_wheel collision surfaces (mu=1.0)
- `<gazebo>` DiffDrive plugin: joints base_link_to_left/right_wheel, separation=0.25 m,
  radius=0.03 m, tf_topic/odom_topic/cmd_vel topics set for ros_gz_bridge mapping,
  odom_frame=odom, robot_base_frame=base_footprint
- `<gazebo reference="laser">` gpu_lidar sensor: 360 samples, 10 Hz, range 0.12–12 m,
  topic=/scan_gz
- `<gazebo>` JointStatePublisher plugin
**Test**: `ros2 run xacro xacro config/dome3_sim.urdf` produces valid XML with no errors.
**Bug found + fixed (2026-07-01)**: sensor was authored with `type="lidar"` instead of
`type="gpu_lidar"`. Gazebo Harmonic's `Sensors` system only instantiates the
rendering-based `gpu_lidar` sensor class (`libgz-sensors8-gpu_lidar`); the plain
`lidar` type (`libgz-sensors8-lidar`, physics-raycast based) is a separate class that
the Sensors system does not drive, so the sensor silently never published — no error,
just an absent topic on the native `gz topic -l` bus. Fixed by changing the `type`
attribute to `gpu_lidar` in `config/dome3_sim.urdf`. Confirmed working manually via
`test1.bash` (Gazebo + robot spawn only, no ROS bridge): lidar rays now visible in the
GUI entity-tree visualization after the fix.

## T03 — Create sim_explore.launch.py
**Status**: done
**Description**: New file `launch/sim_explore.launch.py` using `better_launch` conventions.
Requires `map_name` arg (error if missing). Includes in order:
`ros_gz_sim` GzServer + GzClient (world=`simple_room.world`),
`ros_gz_bridge` for `/scan` (LaserScan), `/odom` (Odometry), `/cmd_vel` (Twist), `/clock`,
slam_toolbox online_async (`use_sim_time: true`),
Nav2 stack (`use_sim_time: true`, reusing `nav2_param_patch.yaml`),
`pluggable_explore_manager_node` (`use_sim_time: true`, reusing `explore_param_patch.yaml`).
All nodes receive `use_sim_time: true`.
**Test**: `bl dome_nav sim_explore.launch.py --map_name sim_test` launches without
errors; `/map`, `/scan`, `/odom`, `/clock`, `/explore/status` all present within 15 s.
Confirmed 2026-07-01: full stack launches, Nav2 lifecycle_manager activates all
servers (controller, planner, route, behavior, bt_navigator, waypoint_follower,
velocity_smoother, collision_monitor, docking_server), slam_toolbox writes
`sim_test.posegraph` + `sim_test.data` to `~/.dome/slam_maps/`.
**Known flakiness**: `bl.include()` nests a separate ROS2 `LaunchService` per include
(Gazebo's own spawn helper, slam_toolbox, and Nav2 each start one — three nested
launch services in one process). This occasionally races and aborts the whole launch
with `cannot schedule new futures after interpreter shutdown` (seen ~2 times out of
~5 runs during verification). Bisected in isolation: Nav2 alone, slam_toolbox alone,
slam_toolbox+Nav2 together, and Gazebo+slam_toolbox together all ran cleanly every
time; only the full triple combination is racy, and even that succeeded on most
runs. Root cause not fully identified — likely a shared executor/thread-pool
teardown race across the three nested `LaunchService` instances. Workaround for now:
just retry the launch. If this proves common rather than rare, the real fix is to
stop nesting Nav2's `navigation_launch.py` as an in-process `LaunchService` (e.g. run
it via a raw `ros2 launch` subprocess instead) — worth its own task if it recurs.

## T04 — Verify sim_time propagation
**Status**: in progress — found a CPU-starvation performance bug, not yet resolved
**Description**: With sim_explore.launch.py running, confirm that TF timestamps,
costmap stamps, and Nav2 action timestamps are all on sim clock. Check with
`ros2 topic echo /clock` and `ros2 topic echo /tf --once`. A wall-clock node will
show stale or zero-latency stamps — fix any that appear.
**Test**: Manual — no TF warnings about old timestamps in the node logs; costmap
updates visible in RViz synchronized with sim time.
**Findings (2026-07-01)**:
- Fixed a real bug in `config/nav2_param_patch.yaml`: `collision_monitor.FootprintApproach`
  had a static `points` override layered on Nav2's default `action_type: "approach"` config
  (which expects a dynamic `footprint_topic`). The hybrid config made `collision_monitor`
  stop producing `/cmd_vel` output entirely (confirmed `/cmd_vel_smoothed` flowing but
  `/cmd_vel` had zero publishers' worth of messages), and once caused its heartbeat to die,
  triggering a full Nav2 lifecycle shutdown. Fixed by removing the override — this file is
  shared with real-hardware launches, so hardware may have had the same latent bug.
- Added a sim-only speed override in `sim_explore.launch.py` (restores
  `desired_linear_vel: 0.3`, `velocity_smoother` limits `[0.4, 0, 1.9]`) since
  `explore_param_patch.yaml`'s real-hardware speed caps (0.12 m/s, deliberately slow for
  slam_toolbox scan-matching) aren't needed in sim. Does not touch shared config files.
- Even with that override, telemetry (`~/.dome/telemetry/explore-newtest4-*.jsonl`) showed
  a goal only 0.5 m away hit the 25s `GOAL_TIMEOUT_S` with the robot moving only 4 cm.
  `controller_server` logged `Control loop missed its desired rate of 20.0000 Hz. Current
  loop rate is 15-45 Hz` throughout. This VM has only 2 CPU cores (`nproc` = 2) running
  Gazebo physics + software-rendered GUI + `ros_gz_bridge` + slam_toolbox (Ceres solver) +
  the full 12-node Nav2 stack (MPPI is itself compute-heavy) concurrently — likely CPU
  starvation causing MPPI to fall back to near-crawl velocities rather than a config issue.
- Added a `headless` launch arg to `sim_explore.launch.py` (`--headless true` → `gz sim -r -s`,
  server only, no GUI) — partly so RViz2 can be used instead, partly to test whether
  removing GUI rendering load fixes the crawl-speed problem. Not yet retested.
- Unresolved: frontier goals are consistently selected ~0.5 m from the robot even though
  `MIN_FRONTIER_DIST` is configured at 0.8 m. Worth checking `pick_best_frontier` behavior
  on this specific map before closing T04/T05.

## T05 — End-to-end exploration smoke test
**Status**: not done — blocked on T04's CPU-starvation/crawl-speed finding
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
