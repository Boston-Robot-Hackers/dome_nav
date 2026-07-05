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
  server only, no GUI) so RViz2/Foxglove can be used instead of the Gazebo GUI.

**Findings (2026-07-02) — package rebuild requirement discovered**: the workspace uses
colcon copy-install, not symlink-install. Editing `.py`/world files under `src/` has no
effect until `colcon build --packages-select dome_nav` runs — `bl`/`ros2 run` execute the
copies under `install/`. Several confusing "the fix didn't work" moments this session were
actually stale installs. Rebuild after every source edit before testing.

**Findings (2026-07-02) — CPU load quantified and reduced, but not the whole story**:
- `top`/`load average` confirmed real, severe CPU contention: load average 5.4–11 on this
  2-core VM (healthy is ≤2.0), ~90% utilized even at baseline before exploration starts.
- Used direct code instrumentation (`time.perf_counter()` around `explore_tick()`'s phases,
  reverted after use — `ptrace`/`strace`/`py-spy` are all blocked in this sandbox, so live
  profilers aren't available) to prove `pluggable_explore_manager_node`'s own tick logic is
  cheap (0.1–6 ms/call), ruling out the node's own code as the CPU-usage source despite it
  showing 25-27% CPU in `top`.
- Root cause of *that* 25-27%: `/clock` was measured at 350-470 Hz in headless mode, driven
  by `worlds/simple_room.world`'s `max_step_size: 0.001` (1ms physics steps, one `/clock`
  message per step). Every `use_sim_time` node in the stack (12+ nodes) pays a message-
  processing tax for that stream regardless of their own workload. Increased to
  `max_step_size: 0.01` (10ms) — cut `/clock` to ~200 Hz and dropped overall system CPU from
  ~90% utilized (9-11% idle) to ~50% utilized (50% idle). Real, measured, worth keeping.
- **However, robot motion did not improve after the CPU fix** — a fresh test still timed out
  a 0.5 m goal after 25.5 s with only 3.3 cm of movement. So CPU contention, while real, is
  not the direct cause of the crawl/stall.
- **Actual direct cause found**: `collision_monitor` is explicitly stopping the robot.
  Log: `Failed to get "dome2/base_footprint/lidar"->"base_footprint" frame transform:
  Lookup would require extrapolation into the past. Requested time 126.100000 but the
  earliest data is at time 126.600000` immediately followed by `Robot to stop due to
  invalid source.` This is designed safety behavior (stop if sensor data can't be verified
  valid), not a crash. `/cmd_vel` was confirmed reading exactly `0,0,0` while this was
  happening. This ties together the whole session's recurring "Detected jump back in time.
  Clearing TF buffer." warnings — something is delaying processing of the lidar-frame TF
  chain by ≥0.5s relative to its timestamp, and collision_monitor brakes every time that
  gap causes a lookup failure. **Not yet root-caused** — candidates not yet ruled out:
  TF buffer duration vs. actual processing latency, whether `gz_laser_frame_bridge` (the
  static identity transform bridging the gz-renamed lidar frame) is itself contributing,
  or `collision_monitor`'s `transform_tolerance`/`source_timeout` (currently Nav2 defaults,
  0.2s / 1.0s) being too tight for this VM's latency under load.
- Also fixed in passing: `check_goal_redirect()` was recomputing full frontier clustering
  every 2Hz tick unconditionally, even though slam_toolbox's `map_update_interval: 5.0`
  means `/map` only changes every ~5s — ~90% of those recomputes were wasted work on an
  unchanged map. Added `frontier_goal_for_current_map()` in
  `pluggable_explore_manager_node.py`, memoized by map `header.stamp`. Confirmed via the
  same instrumentation: redirect cost dropped from ~4ms (fresh compute) to ~0.1ms (cache
  hit) on all but the first tick after each new map. Kept — it's a correct, low-risk
  efficiency win independent of the collision_monitor finding above.
- Still unresolved: frontier goals are consistently selected ~0.5 m from the robot even
  though `MIN_FRONTIER_DIST` is configured at 0.8 m. Worth checking `pick_best_frontier`
  behavior on this specific map before closing T04/T05.
- Also changed: `slam_manager_node.py`'s `SAVE_PERIOD_SEC` constant is now a declared ROS
  parameter `save_period_sec` (default unchanged at 30.0), set to 120.0 in
  `sim_explore.launch.py` — map-save logging was firing distractingly often during manual
  testing.

**Findings (2026-07-02, continued) — TF-extrapolation theory ruled out; real cause is
costmap inflation near the doorway**:
- Killed two orphaned `static_transform_publisher` processes left running from earlier
  sessions (started 15:17/15:25, no matching Gazebo/bridge alive) before starting a clean
  run — reconfirms the process-hygiene issue is not fully closed even after T03's cleanup-
  pattern fix. Prefer an explicit `ps` audit + `kill -9` over trusting `pkill -f` alone.
- Traced the TF chain for the failing lookup (`dome2/base_footprint/lidar`→`base_footprint`):
  every edge in it (`gz_laser_frame_bridge`'s static transform, plus `robot_state_publisher`'s
  fixed-joint transforms) is published on `/tf_static`, not `/tf`. The only dynamic edge in
  the whole tree is `odom`→`base_footprint` from the DiffDrive plugin, which isn't even part
  of this lookup. A purely time-latency explanation for the earlier "extrapolation into the
  past" error doesn't hold up structurally — the mechanism only makes sense if something
  (most plausibly a duplicate/conflicting `/clock` source from an orphaned process, matching
  the pattern above) triggered tf2's "jump back in time" full-buffer clear, which also wipes
  static entries that are not automatically redelivered to an already-connected listener.
- With stale processes cleaned up, reran the full stack (`test3.bash`, then `test4.bash`) and
  did **not** reproduce the TF-extrapolation error in ~40s of live `/rosout` monitoring across
  multiple runs, despite the crawl/stall symptom (`/cmd_vel` ≈ 0 while `/cmd_vel_smoothed` was
  non-zero) still being present. So TF lag is not the active cause in a clean run.
- **Actual direct cause found**: `controller_server` log showed repeated `bt_navigator`
  recovery sequences (Spin → Wait → BackUp) with `backup failed` / `Collision Ahead - Exiting
  DriveOnHeading` while the robot sat at world position (0.10, 0.15) — inside the interior
  doorway (gap spans x≈0, y −0.3…+0.3). `local_costmap`'s `inflation_radius: 0.2` combined
  with `robot_radius: 0.15` leaves very little genuinely low-cost clearance in a 0.6 m-wide
  doorway once the robot is positioned near either wall segment edge — the BackUp recovery
  behavior's own collision check (also against the local costmap) correctly refuses to reverse
  because inflated cost is immediately behind the robot in that tight space. This is a
  geometry/tuning issue (doorway width vs. inflation_radius + robot_radius), not a TF/timing
  bug. Confirmed independent of CPU load and independent of `/clock` behavior.
- Also observed heavy goal-cancel thrashing from `check_goal_redirect()` (goals repeatedly
  canceled after 0.0s and immediately replaced) compounding the appearance of a stall,
  though the underlying blocker is the inflation issue above.
- **Mitigation applied**: added a `max_frontier_dist` cap (`ExploreParams.max_frontier_dist`,
  default 0.0 = unlimited at the pure-algorithm level; operational default 1.0 m via the
  `pluggable_explore_manager_node` ROS parameter and `sim_explore.launch.py` launch arg) so
  exploration only targets frontiers within ~1 m of the robot, on the theory that shorter
  hops reduce exposure to any single bad costmap region and keep goals closer to what's
  already been scanned. `pick_best_frontier()` in `frontier_explorer.py` gained a `max_dist`
  parameter (mirrors the existing `min_dist`); wired through `frontier_algorithm.py`. Added
  3 new pure-Python tests in `test_frontier_explorer.py` (max_dist filtering); 76/76 pure
  tests pass. **Verified this does not fully resolve the stall**: rerunning with the 1 m cap
  still produced a first goal at (0.284, 0.412) — right at the doorway's wall-segment
  boundary (interior_wall_north starts at y=0.3) — which timed out after 25.2 s with only
  0.08 m of robot movement. The distance cap avoids sending the robot far away, but the very
  first frontier the robot must cross is the doorway itself, so it doesn't sidestep the
  inflation problem. A real fix would need to reduce `local_costmap.inflation_layer.
  inflation_radius` and/or widen the doorway in `worlds/simple_room.world`, or adjust
  `cost_scaling_factor` — not yet attempted.
- Removed the `headless` launch arg from `sim_explore.launch.py` entirely (GUI is now
  needed for this kind of visual inflation/collision debugging, and the arg was newly added
  last session specifically to avoid the GUI). Removed the now-broken `test3.bash` (headless
  + foxglove) and replaced it with `test4.bash` (GUI + foxglove, same topic-presence checks).
- Added `test5.bash` (GUI + bridge + rviz2, no slam/Nav2/explore, for isolating the
  Gazebo/URDF/TF layer). First version passed the URDF inline as a `-p key:=value` CLI arg
  to `robot_state_publisher`, which crashed on startup (`RCLInvalidROSArgsError` — rcl's
  arg parser can't handle multi-line XML) — this is why the robot model never appeared in
  RViz2. Fixed by writing the URDF into a temp YAML params file and using `--params-file`,
  same mechanism `sim_explore.launch.py` uses via `better_launch`'s `params={...}` (which
  does this automatically and invisibly). Verified: `/robot_description` publishes,
  `tf2_echo odom base_link` resolves.
- Added 6 single-job `better_launch` files so each piece of the bare sim/TF/RViz2 stack can
  be started in its own terminal window without slam/Nav2/explore, for manual debugging:
  `sim_gazebo.launch.py`, `sim_spawn.launch.py`, `sim_bridge.launch.py`,
  `sim_robot_state_publisher.launch.py`, `sim_laser_tf.launch.py`, `sim_rviz.launch.py`.
  Each just wraps one node/include from `sim_explore.launch.py` (or `test5.bash`) using the
  same `better_launch` calls, so there's no duplicated logic to drift — they read the URDF
  fresh from disk each run, same as `sim_explore.launch.py`.
- Added `sim_nav2.launch.py` — 7th single-job file, starts the Nav2 stack with the same
  config as `sim_explore.launch.py` (nav2_param_patch + explore_param_patch + sim-only speed
  bump). Requires the other 5 gazebo/bridge/TF launch files already running for valid
  TF/odom; without `sim_slam.launch.py` (not yet added) there's no `/map`, so
  `global_costmap`'s static_layer just waits — Nav2 still activates cleanly. Verified via
  smoke test: all lifecycle-managed servers activate, no crashes.
- **Correction (2026-07-03)**: the above "Nav2 still activates cleanly" claim was wrong for
  standalone `sim_nav2.launch.py` without a slam/localization node. Live-session debugging
  found `planner_server`'s `global_costmap` blocks on activation waiting for a valid
  `base_link → map` TF chain; without `slam_toolbox` (or AMCL) publishing `map → odom`, the
  `map` frame never exists, `planner_server` times out after 60s, and `lifecycle_manager`
  aborts the *entire* bringup on that one failure — leaving every other server stuck
  `inactive` even though they configured fine individually. Also found and killed a stale
  `opennav_docking` process orphaned from a prior day's session, which caused a duplicate
  `/docking_server` node-name collision and a similar first-attempt activation failure
  (`ros2 lifecycle get` calls can hang/misbehave against duplicate-named nodes — useful
  diagnostic signal). Added `sim_slam.launch.py` (8th single-job file, `slam_toolbox`
  online_async, same config as `sim_explore.launch.py`, requires `--map_name`) so the `map`
  frame exists before Nav2 tries to activate.
- Added `sim_explore_node.launch.py` (9th single-job file, `pluggable_explore_manager_node`,
  requires `--map_name`, same `max_explore_radius`/`max_frontier_dist` defaults as
  `sim_explore.launch.py`). Completes the manual single-job debug stack: gazebo, spawn,
  bridge, RSP, laser TF, RViz2, Nav2, slam, explore node — same set of nodes as
  `sim_explore.launch.py`, just split across 9 terminals for step-by-step debugging.
- **Bug found + fixed (2026-07-03)**: first live exploration run on the 9-window stack
  immediately reported "No frontiers found" and exhausted patience within 1 tick — telemetry
  showed `large_clusters: 7` (frontiers of adequate size do exist) but
  `all_cells_too_close: 0`, which looked contradictory. Root cause: `_frontier_diag()` in
  `frontier_explorer.py` only ever checked `min_dist`, never `max_dist` — added when
  `max_frontier_dist` was introduced earlier, but the diagnostic helper wasn't updated to
  match, so it was blind to clusters filtered out for being too *far*. With the operational
  default `max_frontier_dist=1.0` and `min_frontier_dist=0.8`, the valid target band is only
  0.2 m wide; in this 4x4 m room a single lidar scan reveals most of the open area
  immediately, so the nearest real frontier (room/doorway boundary) is very likely to land
  outside that narrow band — the diag just couldn't say so. Fixed: `_frontier_diag()` now
  takes `max_dist` too, renamed the counter `all_cells_too_close` → `all_cells_out_of_range`
  (covers both filters), added `_cell_out_of_range()` helper, wired through
  `frontier_algorithm.py`. `explore_manager_node.py` (original, untouched) still calls
  `_frontier_diag()` with its old 5-arg signature — `max_dist` defaults to 0.0 there, so
  nothing changes for it. Added a regression test in `test_frontier_explorer.py`. 155/155
  pure tests pass. **Not yet fixed**: the underlying practical problem — `max_frontier_dist:
  1.0` can be too restrictive for small rooms and may need to be raised (or explicitly
  overridden via `--max_frontier_dist` on `sim_explore_node.launch.py`) to get exploration
  moving at all in `simple_room.world`.
- **Simplified (2026-07-03)**: consolidated the 9 single-job files down to 4, per user
  request. `sim_robot.launch.py` = `sim_gazebo` + `sim_spawn` + `sim_bridge` +
  `sim_robot_state_publisher` + `sim_laser_tf` (gazebo, spawn, bridge, RSP, laser TF — one
  window for a visible, TF-correct robot). `sim_nav.launch.py` = `sim_slam` + `sim_nav2`,
  in the dependency order live-debugging established (slam before Nav2, so `planner_server`
  finds the `map` frame immediately instead of blocking 60s and aborting bringup). Deleted
  the 7 superseded files: `sim_gazebo.launch.py`, `sim_spawn.launch.py`,
  `sim_bridge.launch.py`, `sim_robot_state_publisher.launch.py`, `sim_laser_tf.launch.py`,
  `sim_slam.launch.py`, `sim_nav2.launch.py`. Kept `sim_rviz.launch.py` and
  `sim_explore_node.launch.py` separate (toggled independently of the rest). Smoke-tested
  the new pair end-to-end: `sim_robot.launch.py` → `/robot_description` publishes,
  `tf2_echo odom base_link` resolves; `sim_nav.launch.py` → `/map` publishes,
  `lifecycle_manager` log shows `Managed nodes are active` for all 10 servers (confirms the
  slam-before-nav2 fix). 155/155 pure tests pass after rebuild.
- **Nav2 lifecycle-abort bug recurred, split back into two files (2026-07-04)**: live
debugging found `sim_nav.launch.py`'s two `bl.include()` calls (slam then Nav2) only
guarantee launch *order*, not readiness — `bl.include()` returns as soon as it registers
the nested launch, it does not block until the included stack is actually up. Confirmed via
a live run: Nav2 processes ran for 110s, `/map` and `map→odom` TF were present, but
`bt_navigator`/`controller_server`/`planner_server`/`behavior_server` were all permanently
`inactive` — `planner_server` had already hit its ~60s activation timeout waiting for the
`map` frame (which didn't exist yet when Nav2 started) before slam finished initializing,
and `lifecycle_manager` aborted the whole bringup, matching T04 finding #1 exactly. This is
the same bug as before, just re-triggered because "start slam first" doesn't wait for slam
to be *ready* first. Split `sim_nav.launch.py` back into `sim_slam.launch.py` (slam_toolbox
only) and `sim_nav2.launch.py` (Nav2 only, no `map_name` arg needed) so they can be started
as two separate manual steps with a human-verified pause between them (confirm `/map` is
publishing before starting `sim_nav2.launch.py`), instead of one script racing both. Not yet
verified end-to-end with the split files. The permanent fix (blocking on `/map` inside a
single script before including Nav2) is still open — see Likely next steps.

**`MIN_FRONTIER_DIST` raised 0.8→1.3 (2026-07-03)**, at explicit user request: "never ask
  Nav2 to go to a point closer than a full meter away." The filter runs on the raw frontier
  cell distance, before `GOAL_INSET_M` (0.3 m) pulls the actual sent goal closer — so the
  real floor on the Nav2 goal is `min_frontier_dist - GOAL_INSET_M`, and 1.3 m gives exactly
  the requested 1.0 m floor. Changed in both `ExploreParams` (`explore_context.py`, affects
  the pluggable/sim node) and `explore_manager_node.py`'s `MIN_FRONTIER_DIST` constant (real
  robot) — user explicitly chose to update both when asked, a deliberate one-off exception
  to `explore_manager_node.py` normally staying untouched. 155/155 tests still pass (no
  hardcoded 0.8 assumptions in the test suite). See `02-doc/current.md`'s Exploration params
  section and `01-literate/07-explore_manager_node.md`'s Observations for the full rationale.

## T04a — Fix stray empty map-name directory in ~/.dome/slam_maps
**Status**: done
**Description**: `sim_slam.launch.py` and `sim_explore.launch.py` both ran
`os.makedirs(slam_map_path, exist_ok=True)` where `slam_map_path` is
`~/.dome/slam_maps/<map_name>` — the same string then passed to slam_toolbox as
`map_file_name`. slam_toolbox treats that as a file prefix and writes sibling
`<map_file_name>.posegraph`/`.data` files, never writing into the directory itself,
so every sim run left behind a permanently empty `~/.dome/slam_maps/<map_name>/`
directory. The real-robot launch files (`robot_map.launch.py`,
`robot_explore.launch.py`) don't have this bug — they only
`os.makedirs(home, exist_ok=True)` (the parent `slam_maps/` dir). Fixed both sim
launch files to match that pattern, and removed the existing stray empty
directories from `~/.dome/slam_maps/`.
**Test**: Manual — after fix, run `sim_slam.launch.py --map_name t04afix`, confirm
`~/.dome/slam_maps/t04afix.posegraph`/`.data` are written and no
`~/.dome/slam_maps/t04afix/` directory is created.

## T04b — Create sim_nav_full.launch.py (single-command full stack, composed)
**Status**: done
**Description**: New `launch/sim_nav_full.launch.py` gives back a one-command way
to start the full sim stack, without reintroducing the duplicated logic that
`sim_explore.launch.py` still carries. Built by calling `bl.include("dome_nav",
"sim_robot.launch.py")`, then `sim_slam.launch.py`, `sim_nav2.launch.py`,
`sim_explore_node.launch.py` in that order — `bl.include()` on a `better_launch`
file execs it in the same process sharing the `BetterLaunch` singleton, and
auto-forwards the calling launch's own args to the included function filtered by
its signature (confirmed by reading `better_launch/wrapper.py`'s
`_launch_this_wrapper`: "Pass only those arguments that actually match the
function's signature"), so `map_name` reaches `sim_slam`/`sim_explore_node`
without needing to be re-specified per include. Requires `map_name`; `--headless`
not supported (dropped previously, GUI needed for costmap debugging).
`sim_rviz.launch.py` intentionally left out — kept as a separate optional window,
same as with the split files.
**Test**: `bl dome_nav sim_nav_full.launch.py --map_name t04bcheck` launches
Gazebo, `ros_gz_bridge`, `robot_state_publisher`, laser TF, slam_toolbox, Nav2,
and `pluggable_explore_manager_node` from one command; confirmed `/map`, `/scan`,
`/odom` publish and `lifecycle_manager` reports all Nav2 servers active.

## T04c — Fix missing left_wheel/right_wheel TF (joint_states remap dead)
**Status**: done
**Description**: RViz2's RobotModel display reported no transform available for
`left_wheel`/`right_wheel` (the two `continuous` joints in `dome3_sim.urdf`,
whose TF depends on live joint angles rather than a fixed offset).
Root-caused: `sim_robot.launch.py` (and the duplicated block in
`sim_explore.launch.py`) bridges Gazebo's `JointStatePublisher` plugin output
via `GazeboBridge("/model/dome2/joint_state", ..., remaps={"/model/dome2/
joint_state": "/joint_states"})`, intending the bridge to publish under
`/joint_states` so `robot_state_publisher` picks it up. Traced
`spawn_topic_bridge()` in the installed `better_launch` package
(`better_launch/gazebo.py`): it always starts the bridge node via
`bl.node(..., raw=True)`, and `raw=True`'s own docstring says it "avoid[s]
passing it any command line arguments except those specified" — so the
`remaps` dict it builds is computed but never reaches the process. Confirmed
via `/proc/<pid>/cmdline` on the running bridge: no `-r` remap args present at
all. Result: Gazebo's joint data was real and correct (captured a message with
`base_link_to_left_wheel`/`base_link_to_right_wheel` positions) but published
only under `/model/dome2/joint_state`, which nothing subscribed to —
`/joint_states` had zero publishers, so `robot_state_publisher` never computed
those two transforms. Fixed by remapping the other side instead:
`robot_state_publisher`'s own `bl.node()` call now takes `remaps={"/joint_states":
"/model/dome2/joint_state"}` (a normal, non-`raw` node, where `bl.node()`'s
remaps do work) in both `sim_robot.launch.py` and `sim_explore.launch.py`.
Removed the now-dead `remaps=` kwarg from the `GazeboBridge` joint_state entry
in both files, since it had no effect and would mislead future readers.
**Test**: Manual — after fix, `ros2 topic info /joint_states -v` shows a
publisher (previously zero); `ros2 run tf2_ros tf2_echo base_link left_wheel`
resolves (previously "frame does not exist"); RViz2 RobotModel shows no
missing-transform warning for `left_wheel`/`right_wheel`.

## T04d — Fix empty [min_frontier_dist, max_frontier_dist] band (always-idle bug)
**Status**: done
**Description**: Telemetry (`~/.dome/telemetry/explore-zoo1-20260704.jsonl`)
showed every exploration session ending immediately: `{"reason": "idle",
"goals_sent": 0, "reached": 0, "failed": 0}`. Root cause: `ExploreParams.
min_frontier_dist` defaults to 1.3 (`explore_context.py:20`, raised from 0.8 on
2026-07-03 for the real-robot "never send a goal closer than 1m" request — see
`02-doc/current.md`'s Exploration params section), but `max_frontier_dist`'s
sim-side default stayed at 1.0 — both `pluggable_explore_manager_node.py`'s
ROS parameter (`declare_parameter("max_frontier_dist", 1.0)`) and every sim
launch file's `max_frontier_dist: float = 1.0` argument default
(`sim_nav_full.launch.py`, `sim_explore.launch.py`, `sim_explore_node.launch.py`).
`pick_best_frontier()`'s filter (`frontier_explorer.py:124-126`) skips any cell
with `d < min_dist` (too close) OR `d > max_dist` (too far) — with
min(1.3) > max(1.0), no distance satisfies both, so it always returns `None`
regardless of the map. This is an empty-range bug, not a map/tuning issue; the
2026-07-03 min-dist raise was never propagated to the sim-side max-dist
default. Fixed by raising the sim-side `max_frontier_dist` default to 3.0 (room
is 8x8 m; a single scan reveals most of one side, so the nearest real frontier
after crossing the doorway is likely well past 1.3 m) in
`pluggable_explore_manager_node.py` and all three sim launch files.
**Test**: Regression test added in `test_pluggable_explore_manager_node.py`
asserting the default `max_frontier_dist` parameter is greater than
`ExploreParams.min_frontier_dist`, so this can't silently regress again.
Manual: rerun `sim_nav_full.launch.py`, publish `exploration_start`, confirm
telemetry shows `goals_sent > 0` instead of immediate idle.

## T04e — Add prefer_farthest frontier-selection mode
**Status**: done
**Description**: Live telemetry (`~/.dome/telemetry/explore-zoo2-*.jsonl`) showed
exploration cycling through 6+ failed goals, all clustered within ~1-1.2m of
the robot's start near the interior doorway — each timeout blacklists only a
0.5m radius (`blacklist_radius`) around the failed point, so the next
nearest-unblacklisted cell is often still in the same local obstacle cluster.
Root cause of the *retry-nearby* behavior (distinct from the already-documented
inflation/BackUp stall itself, F13 T04 finding #5b): `pick_best_frontier()`
always selected the single closest candidate cell to the robot
(`frontier_explorer.py`), so repeated failures near one obstacle kept
re-targeting that same neighborhood instead of trying a genuinely different
part of the map. Per explicit user decision this session: sim and real-robot
code must stay identical, differing only by parameter values — so this is
implemented as a new opt-in parameter, not a separate code path.
Added `prefer_farthest: bool = False` to `ExploreParams`
(`explore_context.py`) and `pick_best_frontier()` (`frontier_explorer.py`) —
when `True`, both the per-cluster and cross-cluster selection track the
*farthest* qualifying cell instead of nearest (flips the comparison operator;
all existing filters — blacklist, `min_dist`/`max_dist`, `max_radius` — still
apply first). Wired through `frontier_algorithm.py` and exposed as a ROS
parameter `prefer_farthest` on `pluggable_explore_manager_node.py` (default
`False`, matching `ExploreParams`, so `explore_manager_node.py`'s call sites
and any other caller are unaffected). All three sim launch files
(`sim_nav_full.launch.py`, `sim_explore.launch.py`,
`sim_explore_node.launch.py`) default their `prefer_farthest` launch arg to
`True`, since that's where the retry-nearby problem was observed; real-robot
launch files are untouched and keep the `False` default.
**Test**: 4 new pure tests in `test_frontier_explorer.py` (farthest across
clusters, farthest within one cluster, blacklist still respected, `max_dist`
still respected) + 2 integration tests in `test_frontier_algorithm.py`
confirming `ExploreParams.prefer_farthest` is correctly plumbed through
`FrontierAlgorithm.next_goal()`. 162/162 pure tests pass (up from 156).
**Not yet done**: live re-verification in Gazebo that this actually reduces
the retry-nearby pattern — the underlying costmap-inflation stall (F13 T04
finding #5b) is unfixed and will still cause any individual farther goal to
potentially fail the same way if its path also crosses the doorway; this
task only changes which frontier gets tried next, not whether the robot can
reach it.

## T04f — Disable mid-navigation redirect under prefer_farthest
**Status**: done
**Description**: Live testing (2026-07-05) of T04e's `prefer_farthest` showed
the robot ping-ponging between two points ~1.7m apart instead of exploring —
confirmed via telemetry (`~/.dome/telemetry/explore-boo1-20260705.jsonl`): 17
`goal_sent`/`redirect` pairs alternating between the same two coordinates
every ~10s, zero goals reached. Root cause: `check_goal_redirect()` re-picks
"the best frontier" every tick from the robot's *current* position and
cancels/redirects if it shifted more than `REDIRECT_THRESHOLD` (1.5m) from the
active goal — a mechanism designed to catch genuine map updates revealed by
the lidar during transit. Under `prefer_farthest`, "best" means "farthest from
the robot's current position," which flips to the opposite side as soon as the
robot makes any progress toward its goal — an artifact of the robot's own
motion, not new map information. This is structurally different from
nearest-first, where moving toward the nearest frontier keeps it nearest (or
the map reveals it and a new nearby one takes over), so redirect is stable
there. Fixed by returning early from `check_goal_redirect()` when
`self.prefer_farthest` is `True` — once a farthest-first goal is sent, the
robot commits to it rather than re-evaluating mid-flight. Reasoning documented
inline in the method's comment block, including the specific telemetry file
that demonstrated the failure.
**Test**: 2 new tests in `test_pluggable_explore_manager_node.py` —
`test_redirect_fires_when_not_prefer_farthest` (existing behavior unchanged)
and `test_redirect_suppressed_when_prefer_farthest` (new guard). 164/168 pure
tests pass (up from 162). Not yet re-verified live in Gazebo.

## T04g — Expose min_frontier_size as a ROS parameter, try 1 in sim
**Status**: done
**Description**: User observed (looking at RViz2's Map display, confirmed to
exactly match a direct `/map` topic dump) that visibly farther frontier cells
existed but weren't being picked despite `prefer_farthest`. Investigated by
running `find_frontier_clusters` directly against the live map: 32 clusters
total, but only 2 exceeded `min_frontier_size` (10) — a 363-cell cluster
(cells 0.41-1.51m away) and a 77-cell cluster (0.60-1.24m away, and entirely
excluded anyway since its farthest cell is below `min_frontier_dist`'s 1.3m
floor). The other 30 clusters were 1-4 cells each, reaching up to 2.05m away
— likely real frontier slivers along walls/corners, not necessarily noise —
all discarded by the size filter regardless of distance. This explained why
`prefer_farthest` topped out around 1.5m: every farther candidate was being
filtered out before farthest-selection ever saw it. To test the hypothesis,
exposed `min_frontier_size` as a ROS parameter on
`pluggable_explore_manager_node.py` (default 10, matching `ExploreParams`,
so real-robot behavior is unaffected) and set the sim launch files
(`sim_nav_full.launch.py`, `sim_explore.launch.py`, `sim_explore_node.launch.py`)
to default it to 1, effectively disabling the size filter for this
experiment.
**Test**: 2 new tests in `test_pluggable_explore_manager_node.py` confirming
the ROS parameter default matches `ExploreParams` and is correctly plumbed
into `self.params`. 166/170 pure tests pass (up from 164). Not yet observed
live whether `min_frontier_size=1` actually lets farthest-first reach the
2m-class clusters, or whether those turn out to be scan noise that make
exploration worse (chasing single-cell targets). Follow up after a live run.

## T05 — End-to-end exploration smoke test
**Status**: not done — blocked on T04's doorway costmap-inflation finding (robot cannot
reliably cross the interior doorway; recovery behaviors fail there too)
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
