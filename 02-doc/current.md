# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-07-08
**Branch:** main
**Status (2026-07-08 update):** Sim exploration now works **notably better** — goals are
sent and reached, the map fills in, the earlier "sits and never moves" cases are largely
gone. This session was behavior fixes + a large cleanup:
- **Removed the startup 360° spin** (was T04q) from `pluggable_explore_manager_node.py`.
- **`min_frontier_dist` lowered to 0.9 m in sim** (real stays 1.3) — fixes the startup
  deadlock where the only adequately-sized frontier was < 1.3 m away, so no goal was ever
  sent (confirmed via `wed3` telemetry). Now a ROS parameter on the node.
- **Converged sim and real on one explorer**: `robot_explore.launch.py` now runs
  `pluggable_explore_manager_node` (real-robot param values); the original
  `explore_manager_node.py` and its test/entry/literate doc were **deleted**.
- **Removed ALL YAML patching.** `config/` is now six standalone, commented copies of the
  upstream defaults — `slam_real.yaml`, `slam_sim.yaml`, `nav2_real.yaml`,
  `nav2_localization_real.yaml`, `nav2_explore_real.yaml`, `nav2_explore_sim.yaml`. Deleted
  `slam_param_patch.yaml`, `nav2_param_patch.yaml`, `nav2_amcl_patch.yaml`,
  `empty_dock_database.yaml`, and the helpers `build_slam_config`, `patch_dock_db`,
  `yaml_override`, `yaml_patch_dict`, `deep_merge`, `SIM_SLAM_OVERRIDES`. slam is dropped
  from map_file_name (Option A: `slam_manager_node` still persists per `--map_name`, but
  slam no longer auto-resumes an existing map — re-running a map_name overwrites it). The
  historical docking SIGABRT was root-caused to a `dock_database: ''` artifact; removing it
  to match upstream makes `docking_server` come up `active` (live-verified in sim).
- **`yaw_goal_tolerance` raised to ~π in sim** so exploration goals (sent with a fixed
  identity orientation) don't force a wasteful end-of-goal in-place spin.
- **Remaining nav issues diagnosed but not fully fixed** (documented for a Nav2 post): the
  ~0.11 m/s crawl on short goals is stock MPPI `GoalCritic.threshold_to_consider: 1.4` vs
  ~1 m goals; near-border stalls are the planner reporting "Start occupied" when the robot
  center sits in an inflated/lethal cell; "reversing without turning" is the straight-line
  `BackUp` recovery (a stuck symptom), not a controller bug.
- **Caveat:** real-robot launches (Modes A/B/E) were **not** live-run this session; the
  new `nav2_real.yaml`/`nav2_localization_real.yaml` are byte-faithful copies of the old
  merges (zero behavior change) but unproven on hardware.
See the **Session 2026-07-08** entry under F13 and TF13 tasks T04u–T04x + TF10 T08.
Everything below this point predates this session — historical context only.
**Status (2026-07-07 update, superseded above):** F13 sim stack now boots end-to-end and
reaches the "robot visible in RViz, full Nav2/SLAM/explore stack active" state reliably —
see the **Session 2026-07-07** entry under F13 below for the two real bugs found and fixed
that day (`robot_state_publisher` never starting; a `--param-file`/`--params-file` typo in
`better_launch` itself) plus a race-condition fix, a patience-timing fix, and an
inflation-radius fix.
**Status (as of 2026-07-05, superseded above):** F12 complete. F13 (Gazebo simulation) in progress: T01-T03 done, full sim stack
(Gazebo + slam_toolbox + Nav2 + explore) launches and drives the robot end-to-end. T04's
earlier TF-extrapolation theory was **ruled out** — traced the failing lookup's TF chain and
found every edge in it is static; a clean run (after killing orphaned processes from a prior
session) did not reproduce the TF error at all. The **confirmed cause** of the crawl/stall is
**costmap inflation near the interior doorway**: `local_costmap`'s `inflation_radius: 0.2` +
`robot_radius: 0.15` leaves too little clearance in the 0.6 m-wide doorway, so
`bt_navigator`'s BackUp recovery fails ("Collision Ahead") right where the robot gets stuck.
A `max_frontier_dist` cap was added to reduce goal-hop distance but does **not** fully resolve
this — see F13 T04 for full detail and next steps (reduce inflation_radius and/or widen the
doorway). **New this session (2026-07-03)**: built a set of single-purpose `sim_*.launch.py`
files so the sim stack can be brought up piece-by-piece in separate terminals for manual
debugging (consolidated to 4, then `sim_nav.launch.py` split back into 2 on 2026-07-04 after
a recurrence of the lifecycle-abort bug — see F13 status below for the current 5:
`sim_robot.launch.py`, `sim_slam.launch.py`, `sim_nav2.launch.py`, `sim_rviz.launch.py`,
`sim_explore_node.launch.py`). This live,
piece-by-piece debugging found and fixed two real bugs: (1) Nav2 cannot activate without a
`map` frame — `planner_server` blocks 60s waiting for `base_link → map` TF and
`lifecycle_manager` aborts the *entire* bringup on that one failure, so slam must be launched
before Nav2, not the other way around; (2) `_frontier_diag()`'s telemetry helper only checked
`min_frontier_dist`, never `max_frontier_dist`, so "no frontiers found" telemetry couldn't
explain itself when clusters existed but were all too *far* rather than too close. 155
pytest tests pass (`pytest test/ -m "not manual"`, 4 deselected manual-only). **Reminder:**
this workspace is copy-install, not symlink-install — `colcon build --packages-select
dome_nav` is required after every source edit before `bl`/`ros2 run` will see it.
`tools/algo_demo.py` now has:
- `compound` map (40×40, main room + gap + side corridor + two 4×4 obstacles)
- Line-of-sight sensor reveal via Bresenham ray casting (walls block reveal)
- Travel collision check: straight-line paths blocked by obstacles are blacklisted

## What exists

- `dome_nav/nav_manager.py` — pure Python `NavManager`: JSON parse (uses `"name"` key,
  label from `slots.label`), nearest-target, localization score, status strings
- `dome_nav/frontier_explorer.py` — pure Python frontier detection: OccupancyGrid scan,
  8-connectivity clustering, blacklist-aware nearest-cell selection (NOT centroid),
  max_radius and min_dist filters, `nudge_toward_robot` geometry helper
- `dome_nav/explore_manager_node.py` — **DELETED 2026-07-08** (TF10 T08). The original
  pre-F12 node was orphaned once `robot_explore.launch.py` switched to the pluggable node;
  sim and real now share `pluggable_explore_manager_node.py`.
- `dome_nav/explore_context.py` — **(F12 new)** `ExploreParams`, `ExplorationContext`
  dataclasses and `ExplorationAlgorithm` Protocol
- `dome_nav/frontier_algorithm.py` — **(F12 new)** `FrontierAlgorithm` class wrapping
  the pure frontier functions behind the protocol
- `dome_nav/explore_markers.py` — **(F12 new)** pure functions for RViz `MarkerArray`
  construction (frontiers/blacklist/goal markers); extracted for node file-length budget
- `dome_nav/pluggable_explore_manager_node.py` — the explorer node for **both sim and real**
  (2026-07-08); accepts an injected `ExplorationAlgorithm`. The startup spin was removed
  (2026-07-08), and the mid-navigation redirect (`check_goal_redirect()`/
  `frontier_goal_for_current_map()`/`is_redirecting`/`REDIRECT_THRESHOLD`) — disabled in
  T04s, then **deleted** in the 2026-07-08 cleanup.
- `dome_nav/explore_telemetry.py` — JSONL session logger
- `dome_nav/slam_manager_node.py` — **LifecycleNode**: watches `/map`, saves pose graph
  on first map receipt + every 30s
- `dome_nav/nav_manager_node.py` — ROS2 node: `/intent` → NavigateToPose, status,
  `/amcl_pose` → localization status/score
- `dome_nav/utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()`,
  `write_config()`, **(2026-07-05 new)** `available_worlds()`, `require_world_name()`,
  `world_spawn_xy()` — world-selection validation + per-world spawn point lookup
- `tools/algo_demo.py` — **(F12 new)** interactive CLI demo of `FrontierAlgorithm` on
  hand-crafted ASCII maps. ANSI 256-color; shows clusters A-Z, target T, goal G, robot R,
  blacklist B. Maps: `room`, `corridor`, `ring`, `maze`, `large` (30×30, 3-room layout).
  Now simulates lidar scanning along travel path via `uncover_along_path()` (sweeps at
  radius/2 steps from old to new robot position). Args: `--map`, `--inset`, `--min-size`,
  `--min-dist`, `--sensor-radius`, `--auto`
- `config/` — **(2026-07-08 refactor)** six standalone, commented copies of the upstream
  defaults, no patch chain: `slam_real.yaml`, `slam_sim.yaml`, `nav2_real.yaml`
  (Modes A/B), `nav2_localization_real.yaml` (Mode B AMCL), `nav2_explore_real.yaml`,
  `nav2_explore_sim.yaml`. All the old `*_patch.yaml` files and `empty_dock_database.yaml`
  were deleted; `utils.py` config helpers reduced to `write_config`.
- `launch/robot_map.launch.py` (Mode A), `robot_nav.launch.py` (Mode B),
  `robot_explore.launch.py` (Mode E)
- `launch/sim_explore.launch.py` — **(F13)** full sim stack in one file (Gazebo, bridge,
  RSP, laser TF, slam_toolbox, Nav2, `slam_manager_node`, `pluggable_explore_manager_node`)
- `launch/sim_robot.launch.py`, `sim_slam.launch.py`, `sim_nav2.launch.py`,
  `sim_rviz.launch.py`, `sim_explore_node.launch.py` — **(F13)** the same sim stack split
  into single-purpose files for manual, one-window-per-piece debugging — see F13 status below
- `launch/sim_nav_full.launch.py` — **(F13, 2026-07-04)** single-command full stack,
  composed from the split files above via `bl.include()` instead of duplicating their logic
- `worlds/multi_room.world` — **(F13, 2026-07-05 new)** second world file, 10x10m,
  corner origin (0,0) unlike `simple_room.world`'s centered origin. Floorplan worked out
  interactively with the user via ASCII-diagram iteration before writing SDF: 4x4m room
  (corner, 2m doorway), a whole-box divider wall with a 2m opening, a baffle, and a
  vertical wall segment. Robot spawns at (1,1) in this world (vs. (-1,-1) in
  `simple_room.world`) — see `world_name`/`world_spawn_xy()` below.
- All Gazebo-launching sim files now require `--world_name <simple_room|multi_room>`
  (validated dynamically against `share/dome_nav/worlds/*.world`, not a hardcoded list) —
  see F13 T04j.
- `tools/nav_intent_check.py` — diagnostic: publishes target + intent, verifies nav pipeline

## Tests

| File | Count | Type |
|---|---|---|
| `test_nav_manager_pure.py` | 27 | pure Python |
| `test_utils_pure.py` | 13 | pure Python |
| `test_frontier_explorer.py` | 38 | pure Python |
| `test_frontier_algorithm.py` | 13 | pure Python |
| `test_nav_manager.py` | 19 | ROS mock |
| `test_slam_manager.py` | 11 | ROS lifecycle |
| `test_explore_manager_node.py` | 24 | ROS mock |
| `test_pluggable_explore_manager_node.py` | 29 | ROS mock |
| `test_map_validation.py` | 4 | manual/live only |

**153 pass, 4 deselected** via `pytest test/ -m "not manual"` (as of 2026-07-08, after
deleting `test_explore_manager_node.py` with the orphaned original node, and the
`build_slam_config`/`patch_dock_db` unit tests when YAML patching was removed). The 4
deselected are `test_map_validation.py`'s manual/live-only tests. The table above lists
pre-2026-07-08 per-file counts and is now partially stale (the `test_explore_manager_node.py`
row is gone).

## F12 summary

All tasks done. Architecture is additive — original files untouched.

- **T01** `explore_context.py` — `ExploreParams`, `ExplorationContext`, `ExplorationAlgorithm` Protocol
- **T02** `frontier_algorithm.py` — `FrontierAlgorithm` wraps pure frontier functions
- **T03** `pluggable_explore_manager_node.py` — pluggable ROS2 node (copy-and-modify)
- **T04** `test_frontier_algorithm.py` — 11 pure Python tests (all passing)
- **T05** `test_pluggable_explore_manager_node.py` — ROS mock tests (syntax verified)
- **T06** Full suite regression — 42/42 pure tests pass
- **T07** `tools/algo_demo.py` — interactive CLI demo with color, clusters, 5 maps

**Post-F12 enhancements:**
- `algo_demo.py`: `uncover_along_path()` — sweeps lidar reveal along travel path, not
  just at destination. Step size = sensor_radius/2 ensures full coverage.
- `pluggable_explore_manager_node.py`: `check_goal_redirect()` — mid-navigation
  re-evaluation every tick. Cancels current goal (without blacklisting, via
  `is_redirecting` flag) if best frontier shifts >1.5 m (`REDIRECT_THRESHOLD`).
  Telemetry event: `"redirect"` with old/new goal xy and shift distance.

**algo_demo.py additions (2026-06-28):**
- `compound` map — 40×40, main room (cols 0–28) with vertical wall at col 29, 10-row
  gap (rows 15–24) opening into a 10-col corridor, two 4×4 internal obstacles.
- `bresenham_cells` + `has_line_of_sight` — Bresenham ray casting; walls and obstacles
  block sensor reveal. Previously all unknown cells within radius were revealed.
- Travel collision check in main loop — straight-line path blocked by obstacle causes
  goal to be blacklisted rather than robot teleporting through the wall.
- `01-literate/10-algo_demo.md` — new literate doc for algo_demo.py.

**Future directions noted in `02-doc/notes.md`**:
- `CostmapFrontierAlgorithm` using `/global_costmap/costmap` instead of raw `/map`
- Adaptive goal distance (prefer far frontiers when near ones are on the travel path)
- Directional continuity bonus (discount frontiers already covered by path scanning)

## F13 — Gazebo Simulation (in progress)

Feature file: `03-features/notdone/F13-gazebo-simulation.md`
Task file: `04-tasks/notdone/TF13-gazebo-simulation.md`

Goal: run full Mode E exploration stack (slam_toolbox + Nav2 + pluggable explore node)
inside Gazebo Harmonic (`gz sim`) on a dev machine. `linorobot2_gazebo` ruled out (needs
Gazebo Classic, not available on Jazzy). New deliverables: `launch/sim_explore.launch.py`,
`worlds/simple_room.world`, `config/dome3_sim.urdf`. No new Python source in `dome_nav/`.

**Status (2026-07-01)**:
- T01 (simulator choice) — done: Gazebo Harmonic 8.11.0 confirmed, `linorobot2_gazebo`/Classic ruled out.
- T02a (`worlds/simple_room.world`) — done: 8×8 m room, interior wall + doorway.
- T02b (`config/dome3_sim.urdf`) — done: inertia, friction, DiffDrive, gpu_lidar, JointStatePublisher.
  Bug found + fixed: sensor was authored `type="lidar"` instead of `type="gpu_lidar"` — Gazebo
  Harmonic's `Sensors` system only drives the rendering-based `gpu_lidar` class, so the sensor
  silently never published (no error, just an absent topic). Fixed; confirmed working via
  `test1.bash` (standalone Gazebo + robot spawn, no ROS bridge/nav stack) — lidar rays now
  visible in the GUI entity-tree visualization.
- T03 (`launch/sim_explore.launch.py`) — done. Confirmed 2026-07-01: full stack (Gazebo,
  `ros_gz_bridge`, slam_toolbox, Nav2, `slam_manager_node`, `pluggable_explore_manager_node`)
  launches; Nav2's lifecycle_manager activates all servers; slam_toolbox writes
  `sim_test.posegraph` + `sim_test.data` to `~/.dome/slam_maps/`.
  **Known flakiness**: `bl.include()` nests a separate ROS2 `LaunchService` per include —
  Gazebo's spawn helper, slam_toolbox, and Nav2 each start one, so three nested launch
  services run concurrently in one process. This occasionally races and aborts with
  `cannot schedule new futures after interpreter shutdown` (~2 of 5 verification runs).
  Bisected in isolation: Nav2 alone, slam_toolbox alone, slam+Nav2 together, and
  Gazebo+slam together all ran clean every time; only the full triple combination is
  racy, and even that succeeded most runs. Workaround: retry the launch. If this recurs
  often, the real fix is to stop nesting Nav2's `navigation_launch.py` in-process (run it
  as a raw `ros2 launch` subprocess instead) — see TF13 T03 notes.
- **New launch arg**: `sim_explore.launch.py` now accepts `--headless true` (adds `-s` to
  `gz_args` so `gz sim` runs server-only). Added so RViz2/Foxglove can be used instead of
  the Gazebo GUI.
- **Bug found + fixed**: `config/nav2_param_patch.yaml`'s `collision_monitor.FootprintApproach`
  override added a static `points` polygon on top of Nav2's default `action_type: "approach"`
  config, which expects a dynamic `footprint_topic` instead. The hybrid config caused
  `collision_monitor` to stop producing output on `/cmd_vel` (confirmed via `/cmd_vel_smoothed`
  publishing fine while `/cmd_vel` had zero messages), and in one run it caused
  `collision_monitor`'s heartbeat to die entirely, triggering a full Nav2 lifecycle
  shutdown ("CRITICAL FAILURE: SERVER collision_monitor IS DOWN"). Fixed by removing the
  override entirely (inherits Nav2's tested default). This file is shared with the real
  robot launches, so real hardware may have had the same latent bug.
- **Sim-only speed override** added in `sim_explore.launch.py` (after the shared
  `nav2_param_patch.yaml` + `explore_param_patch.yaml` merge): restores
  `desired_linear_vel: 0.3` and `velocity_smoother` limits to `[0.4, 0, 1.9]` for sim
  testing only, since `explore_param_patch.yaml`'s conservative real-hardware speed caps
  (`desired_linear_vel: 0.12`, deliberately slow to protect slam_toolbox scan-matching)
  aren't needed in sim. Does not touch the shared real-hardware config files.
- T04 (sim_time propagation) — **in progress; root cause of the crawl/stall now found**.
  Full investigation chain (2026-07-02), see TF13 T04 for complete detail:
  1. **Package rebuild requirement discovered**: workspace is copy-install, not
     symlink-install. Source edits need `colcon build --packages-select dome_nav` before
     `bl`/`ros2 run` will see them — several early "the fix didn't work" moments this
     session were actually stale installs.
  2. **CPU load quantified**: load average 5.4–11 on this 2-core VM (~90% utilized) even
     at baseline. Used direct `time.perf_counter()` instrumentation (reverted after use —
     `ptrace`/`strace`/`py-spy` are all blocked in this sandbox) to prove
     `pluggable_explore_manager_node`'s own tick logic is cheap (0.1–6ms), ruling out its
     own code despite showing 25-27% CPU in `top`.
  3. **Real cause of that 25-27%**: `worlds/simple_room.world`'s `max_step_size: 0.001`
     (1ms physics steps) meant `/clock` published at 350-470Hz, and every `use_sim_time`
     node (12+) pays a processing tax for that stream. Increased to `0.01` (10ms) → `/clock`
     down to ~200Hz, system CPU down from ~90% utilized to ~50% utilized. Real, measured,
     kept.
  4. **But motion didn't improve after the CPU fix** — proving CPU contention, while real,
     wasn't the direct cause of the stall.
  5. **TF-extrapolation theory (superseded, see below)**: earlier in this investigation,
     `collision_monitor` was seen deliberately stopping the robot — `Failed to get
     "dome2/base_footprint/lidar"->"base_footprint" frame transform: Lookup would require
     extrapolation into the past` → `Robot to stop due to invalid source.` — with `/cmd_vel`
     reading exactly `0,0,0`. This session, traced the full TF chain for that lookup
     (`gz_laser_frame_bridge`'s static transform + `robot_state_publisher`'s fixed-joint
     transforms) and found every edge in it is static (`/tf_static`); the only dynamic edge
     anywhere in the tree is `odom`→`base_footprint`, which isn't even part of this lookup.
     Found two orphaned `static_transform_publisher` processes still running from a prior
     session (killed before retesting) — the likely actual mechanism is a duplicate/stale
     `/clock` source triggering tf2's "jump back in time" full-buffer clear (which wipes
     static entries too, since they're not redelivered to an already-connected listener).
     After cleaning up stale processes, reran the stack multiple times and did **not**
     reproduce the TF-extrapolation error at all — so it is not the active cause in a clean
     run, though it's a real failure mode worth guarding against via better process hygiene.
  5b. **Actual confirmed direct cause**: costmap inflation near the interior doorway.
     `controller_server`'s log showed `bt_navigator` running Spin → Wait → BackUp recovery
     with `backup failed` / `Collision Ahead - Exiting DriveOnHeading` while the robot sat at
     (0.10, 0.15) — inside the doorway (gap spans x≈0, y −0.3…+0.3). `local_costmap`'s
     `inflation_radius: 0.2` + `robot_radius: 0.15` leaves very little low-cost clearance in
     the 0.6 m-wide doorway once the robot is near either wall segment edge; BackUp's own
     collision check (against the same local costmap) correctly refuses to reverse because
     inflated cost is immediately behind the robot. This is a geometry/tuning issue (doorway
     width vs. inflation_radius + robot_radius), confirmed independent of CPU load and TF
     timing. Not yet fixed — candidates: reduce `local_costmap.inflation_layer.
     inflation_radius`, widen the doorway in `worlds/simple_room.world`, or reduce
     `cost_scaling_factor`.
  6. **Fixed in passing**: `check_goal_redirect()` recomputed full frontier clustering
     every 2Hz tick unconditionally, even though slam_toolbox's `map_update_interval: 5.0`
     means `/map` only changes every ~5s (~90% of recomputes were wasted). Added
     `frontier_goal_for_current_map()`, memoized by map `header.stamp` — confirmed via the
     same instrumentation (~4ms → ~0.1ms on cache hits). Kept — correct, low-risk win,
     independent of the collision_monitor finding.
  7. **Also changed**: `slam_manager_node.py`'s `SAVE_PERIOD_SEC` is now a declared
     parameter `save_period_sec` (default unchanged, 30.0), set to 120.0 in
     `sim_explore.launch.py` — map-save logging fired too often during manual testing.
- **`max_frontier_dist` cap added** (2026-07-02): `pick_best_frontier()` in
  `frontier_explorer.py` gained a `max_dist` parameter (mirrors existing `min_dist`), wired
  through `ExploreParams.max_frontier_dist` (dataclass default 0.0 = unlimited) and a new
  `pluggable_explore_manager_node` ROS parameter / `sim_explore.launch.py` launch arg (both
  default 1.0 m) — exploration now only targets frontiers within ~1 m of the robot. 3 new
  pure tests added; 76/76 pure tests pass. **Does not fully resolve the doorway stall**:
  reverified with the cap active — first goal (0.284, 0.412), right at the doorway's
  wall-segment boundary, still timed out after 25.2s with only 0.08 m of movement. The cap
  avoids long hops but doesn't route around the doorway, which is the first frontier the
  robot must cross regardless.
- T05 (end-to-end exploration smoke test), T06 (docs/move to done) — blocked on resolving
  the doorway costmap-inflation stall above (5b).

**Session 2026-07-04 (continued) — sim_nav_full.launch.py, 3 real bugs found and fixed via
live testing, prefer_farthest algorithm change**:
- **T04b**: Added `launch/sim_nav_full.launch.py` — single-command full sim stack, composed
  from the existing split files (`sim_robot`, `sim_slam`, `sim_nav2`, `sim_explore_node`) via
  `bl.include()` rather than duplicating their logic like `sim_explore.launch.py` does.
  `better_launch`'s `bl.include()` execs a `better_launch`-style file in-process sharing the
  `BetterLaunch` singleton, and auto-forwards the calling launch's own args to each included
  function's signature (confirmed by reading `better_launch/wrapper.py`'s
  `_launch_this_wrapper`). Smoke-tested end-to-end: all Nav2 servers activated, explore node
  started, args (`map_name` etc.) correctly reached every included file.
- **T04c**: Found and fixed a real TF bug via live user testing — RViz2 reported no transform
  for `left_wheel`/`right_wheel` (the two `continuous` joints). Root cause:
  `spawn_topic_bridge()` in the installed `better_launch` package always starts the Gazebo
  bridge node with `raw=True`, which (per its own docstring) drops any `remaps` passed to it
  — confirmed via `/proc/<pid>/cmdline` showing zero `-r` args on the running bridge process.
  So `GazeboBridge("/model/dome2/joint_state", ..., remaps={...: "/joint_states"})` in
  `sim_robot.launch.py`/`sim_explore.launch.py` never took effect; `/joint_states` had zero
  publishers and `robot_state_publisher` never got joint data. Fixed by remapping the other
  side instead — `robot_state_publisher`'s own `bl.node()` call now takes
  `remaps={"/joint_states": "/model/dome2/joint_state"}` (a normal, non-`raw` node, where
  `bl.node()`'s remaps do work). Removed the now-dead `remaps=` from the `GazeboBridge` entry
  in both files.
- **T04d**: Found and fixed an always-idle bug via live telemetry — every exploration session
  ended immediately with `goals_sent: 0`. Root cause: `ExploreParams.min_frontier_dist`
  defaults to 1.3 (raised from 0.8 on 2026-07-03 for the real-robot "never closer than 1m"
  request) but the sim-side `max_frontier_dist` default stayed at 1.0 in
  `pluggable_explore_manager_node.py` and all three sim launch files — an empty
  `[min=1.3, max=1.0]` band that `pick_best_frontier()` can never satisfy, regardless of the
  map. Fixed by raising the sim-side `max_frontier_dist` default to 3.0 everywhere. Added a
  regression test asserting the default `max_frontier_dist` exceeds `ExploreParams.
  min_frontier_dist` so this can't silently regress again.
- **T04e**: User observed (after T04d's fix let exploration actually run) that goals kept
  failing near the doorway and retrying nearby points in the same small area — telemetry
  showed 6+ consecutive failed goals all within ~1-1.2m of each other. Root cause: nearest-
  first frontier selection is structurally biased toward wall-hugging frontier cells (that's
  usually *why* they're still frontiers), and `blacklist_radius` (0.5m) only clears a small
  bubble around each failure, smaller than the zone where costmap inflation makes an approach
  impossible — so "nearest remaining" after a failure was often still in the same
  practically-unreachable band. Per explicit user decision: **sim and real-robot code must
  stay identical, differing only by parameter values** — `explore_manager_node.py` is no
  longer to be treated as a frozen/untouched rollback copy; future work should converge on
  `pluggable_explore_manager_node.py` for both, with real-robot switch-over as a distinct,
  explicitly-confirmed follow-up (not done yet). Implemented as a new opt-in parameter:
  `prefer_farthest: bool = False` on `ExploreParams` and `pick_best_frontier()` — flips
  nearest-first to farthest-first selection (all existing filters — blacklist, min/max dist,
  max_radius — still apply first). Wired through `frontier_algorithm.py`, exposed as a ROS
  parameter on `pluggable_explore_manager_node.py` (default `False`), and defaulted to `True`
  in all three sim launch files. 6 new tests added (162/166 total). Not yet re-verified live
  in Gazebo — and note this changes *which* frontier is tried next, not whether the robot can
  reach it; the underlying doorway inflation stall (5b) is still unresolved.
- Also discussed and explicitly declined this session: a `pick_best_frontier` cost-filter
  variant reading `/global_costmap/costmap` to reject inflated cells (the "Future: costmap-
  based frontier exploration" idea already in `02-doc/notes.md`) — user said no for now.

**Session 2026-07-05 — inflation math worked out by hand, new `multi_room.world`,
`prefer_farthest` debugged through two real bugs**:
- **T04f**: Live-verified `prefer_farthest` and found it ping-ponging between two points
  ~1.7m apart (17 `goal_sent`/`redirect` pairs, 0 reached). Root cause and fix: see
  `prefer_farthest` entry above — `check_goal_redirect()` disabled under `prefer_farthest`.
- **Inflation geometry worked out with the user**: for a zero-cost centerline through a
  passage, `inflation_radius <= (doorway_width / 2) - robot_radius`. For the old
  `simple_room.world` doorway (0.6m) and `robot_radius` (0.15m), that's `<=0.15`. Tried
  lowering `local_costmap.inflation_layer.inflation_radius` 0.2→0.15 and raising
  `cost_scaling_factor` 10→20 (T04h) — but Nav2 itself flagged 0.15 as below the
  footprint's own computed inscribed radius (0.157), and the math shows **no**
  `inflation_radius` value can satisfy both Nav2's safety minimum and this doorway's
  clearance simultaneously (0.6 - 2×0.157 = 0.286m, still less than the ~0.32m footprint
  diameter) — inflation tuning alone cannot fix a doorway this tight; it needs widening.
  Decided to build a new world instead of widening this one; `inflation_radius` reverted
  to 0.2 (safe), `cost_scaling_factor` kept at 20 (general improvement, doorway-independent).
- **T04i — `worlds/multi_room.world` created**: floorplan worked out interactively via
  ASCII-diagram iteration (0.5m/char text grids, refined turn-by-turn) before writing any
  SDF — 10x10m box, corner origin (0,0) (unlike `simple_room.world`'s centered origin), a
  4x4m room with a 2m doorway, a whole-box divider wall with a 2m opening, a baffle, and a
  vertical wall segment. Robot spawns at (1,1). Verified via `gz model --list` (all 12
  models load) since `gz sim -s --iterations 50` hangs in this sandbox for *both* world
  files (pre-existing environment quirk, not a regression).
- **T04j — `world_name` launch argument**: added `available_worlds()`,
  `require_world_name()`, `world_spawn_xy()` to `dome_nav/utils.py` (pure, tested).
  `require_world_name()` raises listing every world actually installed under
  `share/dome_nav/worlds/` (dynamic, not hardcoded) plus a usage hint, when missing/invalid.
  `world_spawn_xy()` maps a world name to its designed spawn point automatically. Wired
  into `sim_robot.launch.py`, `sim_explore.launch.py`, `sim_nav_full.launch.py`. Verified
  live: missing arg raises the listing error; `--world_name multi_room` spawns at exactly
  `(1.0, 1.0)` (confirmed via the actual `ros_gz_sim create -x 1.0 -y 1.0` command line).
- **T04k**: raised sim cruise speed 50% at user request (`desired_linear_vel` 0.3→0.45,
  `velocity_smoother` linear cap 0.4→0.6) in both `sim_nav2.launch.py` and
  `sim_explore.launch.py`. Sim-only; real-robot cap (0.12) untouched.
- **T04l — critical bug found via `multi_room.world` live testing**: `prefer_farthest` +
  `min_frontier_size=1` (T04g) caused a **100% goal failure rate** (24 sent, 0 reached) in
  the bigger world. Root cause from the actual Nav2 logs: `planner_server` repeatedly
  failed the *same* goal with `Failed to create a plan from potential when a legal
  potential was found. This shouldn't happen.` — a known NavFn edge case where the goal
  sits right at the ragged edge between known and unknown space. Confirmed this was not
  the inflation/doorway mechanism, since the user reported the stall reproducing with the
  robot 1m clear of every wall (inflation-based caution requires physical obstacle
  proximity; this only requires a bad goal coordinate). `min_frontier_size=1` let isolated
  single-cell frontier slivers at that ragged edge qualify as `prefer_farthest` targets —
  exactly the pathological NavFn input, and plausibly the same root cause as an earlier
  `worldToMap failed: mx,my: 207,98, size_x,size_y: 202,202` error seen in the same
  investigation (202 cells ≈ this world's ~10m extent; the error fires for points near the
  map's edge). Fixed by raising `min_frontier_size` back to 5 in all three sim launch
  files. **Re-verified live same session (T04m) and found NOT fully fixed**: reviewed
  telemetry (`explore-toy4-20260705.jsonl`, the first run after this fix) plus the matching
  Nav2 logs. Result: 0 of 29 goals reached — same as every other run logged today (toy1-3,
  zoo3, zoo5 all show `reached: 0`). `planner_server`'s log shows the identical "Failed to
  create a plan from potential when a legal potential was found" error recurring for 10
  distinct goal targets in that one run, including a goal 1.55 m from the nearest wall (open
  space, not a ragged-edge or inflation-zone goal). So `min_frontier_size` 1→5 reduced how
  often a pathological goal triggers this NavFn bug, but did not fix the underlying planner
  defect — it still fires on ordinary goals. Because the explore node's own 25s
  `GOAL_TIMEOUT_S` elapses before Nav2's internal retry loop surfaces a hard failure, these
  show up in telemetry as `"timeout"` rather than the loud planner error, which is why the
  original live check (looking only for absence of a crash) appeared to confirm the fix.
  Also quantified: the `worldToMap failed: mx,my: ..., size_x,size_y: 202,202` boundary
  error occurred 4,460 times in this single run — far more than an occasional edge case,
  still unresolved. Separately, cross-checked the "are we always picking frontiers inside an
  obstacle's inflation zone?" hypothesis against `multi_room.world`'s wall geometry: 13 of
  29 goals (45%) landed within `robot_radius + inflation_radius` (0.35 m) of a wall, which
  explains most (8/10) of the fast `aborted`-status failures, but not the `timeout` failures
  (which include clear-space goals) — so inflation-zone placement is a real contributing
  factor but not the dominant cause. `behavior_server`'s log shows every individual
  Spin/Wait/BackUp recovery step completing successfully in this run, confirming this is
  **not** a repeat of the `simple_room.world` doorway BackUp-collision mechanism (T04 5b) —
  it's purely the planner failing to produce a path, recovering, and failing again in a loop
  until timeout. See TF13 T04m for full detail. F13 T05 remains blocked, now on this planner
  defect rather than on goal-placement/doorway geometry.

**Manual single-window debug launch files (2026-07-03)**: built a set of `sim_*.launch.py`
files so each piece of the sim stack can be started in its own terminal, for step-by-step
debugging independent of `sim_explore.launch.py`'s all-in-one behavior. Started at 9 files
(one per node/include), then consolidated to 4 per user request once the granular debugging
had done its job:
- `sim_robot.launch.py` — Gazebo GUI, robot spawn, `ros_gz_bridge`, `robot_state_publisher`,
  and the static gz-laser-frame transform. Everything needed for a visible, TF-correct robot
  with no slam/Nav2/explore.
- `sim_nav.launch.py` — slam_toolbox + Nav2, **in that order** (see below for why order
  matters). Requires `sim_robot.launch.py` running first. Takes `--map_name`.
- `sim_rviz.launch.py` — RViz2 with `use_sim_time` on.
- `sim_explore_node.launch.py` — `pluggable_explore_manager_node`. Requires the above three.
  Takes `--map_name`, `--max_explore_radius`, `--max_frontier_dist`.

Two real bugs found via this live, incremental debugging (neither was visible when only
using the all-in-one `sim_explore.launch.py`, since that file already launches slam before
Nav2 by construction):

1. **Nav2 cannot activate without slam/localization already running.** Standalone
   `sim_nav2.launch.py` (now folded into `sim_nav.launch.py`) got `planner_server` stuck:
   its `global_costmap` blocks on activation waiting for a valid `base_link → map` TF chain,
   which only exists once something (slam_toolbox or AMCL) publishes `map → odom`.
   `lifecycle_manager` timed out after 60s and — critically — **aborted the entire bringup**
   on that one failure, leaving every other server (`controller_server`, `bt_navigator`,
   etc.) stuck `inactive` even though they'd configured fine individually. This produces
   the exact TF error a user would see: `Invalid frame ID "map" ... frame does not exist`.
   Fixed by combining slam+Nav2 into `sim_nav.launch.py` in the correct order — confirmed via
   `lifecycle_manager`'s log (`Managed nodes are active` for all 10 servers) and reproducible
   `active [3]` lifecycle-state queries.
2. **Stale orphaned processes cause node-name collisions across days, not just sessions.**
   Found an `opennav_docking` process still running from the *previous day's* session
   (`ps` showed a start time from `Jul02`), colliding with a fresh run's own `docking_server`
   under the same node name — this made `lifecycle_manager`'s service call land on the wrong
   (already-`active`) node and fail the state transition. This is the same family of bug as
   the earlier `static_transform_publisher` orphans found during the TF-extrapolation
   investigation, but persisting far longer than expected. Confirms the standing lesson:
   always `ps` audit + explicit `kill -9` before trusting a fresh run's process set,
   regardless of how long it's been since the last session.
3. **Bug in `_frontier_diag()` (not process-related)**: found while actually trying to
   explore for the first time. Telemetry showed `large_clusters: 7` (adequately-sized
   frontiers exist) but `all_cells_too_close: 0`, yet `pick_best_frontier()` still returned
   `None` — looked contradictory. Root cause: `_frontier_diag()` in `frontier_explorer.py`
   was only ever updated to check `min_dist`, never `max_dist`, when `max_frontier_dist` was
   added earlier — it was blind to clusters filtered out for being too *far*. With the
   operational default `max_frontier_dist=1.0` and `min_frontier_dist=0.8`, the valid target
   band is only 0.2m wide; in a 4x4m room a single lidar scan reveals most of the open area
   immediately, so the nearest real frontier is very likely outside that narrow band. Fixed:
   `_frontier_diag()` now takes `max_dist` too, renamed `all_cells_too_close` →
   `all_cells_out_of_range` (covers both filters), added a `_cell_out_of_range()` helper,
   wired through `frontier_algorithm.py`. `explore_manager_node.py` (original, untouched)
   still calls it with its old 5-arg signature — `max_dist` defaults to 0.0 there, so nothing
   changes for it. Added a regression test. **Not yet fixed**: the underlying practical
   problem — `max_frontier_dist: 1.0` may be too restrictive for `simple_room.world`; pass
   `--max_frontier_dist 3.0` (or similar) to `sim_explore_node.launch.py` to work around it
   for now.

`test1.bash` (repo root, untracked) — ad hoc manual smoke-test script: starts `gz sim` with
`worlds/simple_room.world` and spawns `dome2` via `config/dome3_sim.urdf`, no ROS bridge or
nav stack. Used to isolate and debug the Gazebo/URDF layer independent of ROS2.

`test2.bash` (repo root, untracked) — runs the full `sim_explore.launch.py` stack headlessly
(`BL_UI_OVERRIDE=disable`, since the TUI needs a real attached terminal and crashes when
backgrounded), kills stale processes from prior runs first, and checks actual publisher
counts (not just topic-list membership) for `/map`, `/scan`, `/odom`, `/clock`,
`/explore/status`. Usage: `./test2.bash [map_name]` (defaults to `sim_test`).
**Cleanup fixed 2026-07-01**: the original cleanup patterns for `robot_state_publisher` and
`static_transform_publisher` matched text *after* the multi-KB embedded URDF blob in the
command line — `pkill -f` can truncate long cmdlines, so those patterns silently never
matched, leaving orphaned processes across runs (this caused a continuous stream of
"jump back in time" TF warnings from stale `/clock`/`/tf` sources with no live Gazebo
behind them). Also added kills for `ros_gz_bridge` and every individual Nav2/`slam_toolbox`
binary, none of which were previously covered. Patterns now match only stable binary paths.

**Headless mode removed (2026-07-02)**: the `headless` launch arg was removed from
`sim_explore.launch.py` entirely — the Gazebo GUI is needed to visually inspect costmap
inflation and robot behavior near obstacles (see doorway-stall finding above). `test3.bash`
(which relied on `--headless true`) was deleted.

`test4.bash` (repo root, untracked) — like `test2.bash` (GUI always on, now the only mode),
plus starts `foxglove_bridge` (`ws://localhost:8765`, `use_sim_time:=true`) after the stack
is up so Foxglove Studio can be used alongside the Gazebo GUI. Kills `foxglove_bridge` from
prior runs at startup and via an exit trap. Usage: `./test4.bash [map_name]` (defaults to
`sim_test`).

`test5.bash` (repo root, untracked) — Gazebo GUI + robot spawn + `ros_gz_bridge` +
`robot_state_publisher` + laser-frame static TF + RViz2, no slam/Nav2/explore. For isolating
the Gazebo/URDF/TF layer with RViz2 visualization. **Bug found + fixed**: the first version
passed the URDF inline as a `-p robot_description:=<xml>` CLI arg to `robot_state_publisher`,
which silently crashed on startup (`RCLInvalidROSArgsError` — rcl's arg parser can't handle
multi-line XML on the command line) — this is why the robot model never appeared in RViz2.
Fixed by writing the URDF into a temp YAML params file and using `--params-file`, the same
mechanism `bl.node(params={...})` uses automatically under the hood in the `sim_*.launch.py`
files (confirmed by inspecting why *those* worked fine with the same multi-line value).
Verified: `/robot_description` publishes, `tf2_echo odom base_link` resolves.

**`test1.bash`/`test2.bash`/`test4.bash`/`test5.bash` deleted (2026-07-04)**, at user
request, now that `sim_*.launch.py` manual debug files cover the same ground. `test3.bash`
was already deleted in a prior session.

**Process-hygiene lesson learned this session**: across many manual test iterations, `pkill
-f` repeatedly, unpredictably failed to match processes that were verifiably running
(confirmed via explicit PID kills succeeding immediately after). Root cause not identified
(possibly a `pkill` version quirk or cmdline-length interaction in this environment beyond
the already-known URDF-blob truncation issue). When cleaning up manually, prefer explicit
`kill -9 <pid>` after a `ps` audit over trusting a single `pkill -f` pass — verify with a
second `ps` check before assuming clean.

Also note: `setup.py` was updated to add the `pluggable_explore_manager_node` console entry
point and to install `worlds/*` as package share data — both required for T03 and already
in place.

**Session 2026-07-07 — root-caused and fixed `robot_state_publisher` never starting
(the actual cause of "robot model does not appear in RViz"), a `better_launch` typo bug,
a Nav2/SLAM startup race, a patience-timing bug, and an inflation-radius performance
issue. See F13 T04t for full detail.**

- **T04t** — Live debugging of a "robot model does not appear in RViz" report found
  `sim_robot.launch.py`'s `bl.node()` calls for `robot_state_publisher` and
  `gz_laser_frame_bridge` never actually started (confirmed via repeated `ros2 node
  list`/`ps aux` checks across many independent runs, never resolving even after minutes).
  Ruled out, in order: URDF/params content, command-line length (params-file vs. inline
  dict), and Gazebo's own process competing for the GIL (reproduced identically with
  Gazebo started completely externally). **Actual root cause**: neither call passed an
  explicit `name=`, so `bl.node()` treated them as anonymous and called
  `get_unique_name()`, which scans *every process on the system* via
  `get_nodes(include_foreign=True)`/`find_foreign_nodes()` to avoid a name collision —
  on this VM (342 processes) that scan effectively never completed. Fixed by adding
  `name="robot_state_publisher"` to that call. Separately found and fixed a second,
  independent bug in `better_launch` itself: `elements/node.py` emitted `--param-file`
  for `param_files` entries, but ROS2's actual `rcl` flag is `--params-file` (plural,
  confirmed via `strings` on `librcl.so`) — this silently broke `param_files` for every
  caller in this workspace's `better_launch`, not just this one. Both fixes rebuilt and
  live-verified: full stack (`robot_state_publisher`, `gz_laser_frame_bridge`,
  `slam_toolbox`, every Nav2 server, `lifecycle_manager_navigation`, `explore_manager`)
  now comes up, robot visible in RViz.
- **Gazebo launch removed from `sim_robot.launch.py`** — no longer calls
  `gazebo.gazebo_launch()`; expects `gz sim -r <world>.world` to already be running,
  started separately by hand. This was explored as a hypothesis for the
  `robot_state_publisher` hang (it wasn't the cause, see above) but kept anyway: running
  Gazebo as a fully independent process simplifies debugging (native `gz topic -l` can
  confirm the sim layer independent of ROS2/`better_launch`). `sim_nav_full.launch.py`
  and its header comment updated to match — Gazebo is now a precondition, not something
  it starts. A `headless` launch arg was added and then fully removed again during this
  same investigation (per explicit user decision to drop headless support entirely and
  return to the GUI-only, Gazebo-started-separately model).
- **`wait_for_map_odom_tf()` added to `sim_nav_full.launch.py`** — fixes a
  previously-known-but-unfixed race (flagged as open back in the 2026-07-04 session
  notes): `bl.include()` only guarantees launch *order*, not *readiness*, so
  `sim_nav2.launch.py` could start before `slam_toolbox` had published its first
  `map→odom` transform. Nav2's `global_costmap` only waits a **hardcoded 0.5s** for that
  transform during activation (confirmed via `strings` on `libnav2_costmap_2d_core.so`;
  not YAML-configurable) and `lifecycle_manager` aborts the *entire* bringup if it times
  out — confirmed live via `planner_server`'s own log: "Failed to activate global_costmap
  because transform from base_link to map did not become available before timeout",
  leaving `global_costmap`, `bt_navigator`, `behavior_server`, `collision_monitor`,
  `docking_server`, `route_server`, `velocity_smoother`, and `waypoint_follower` all
  stuck `inactive` while `controller_server`/`smoother_server`/`local_costmap` (which
  don't need the `map` frame) activated fine. Fixed by blocking on the real transform
  (via `tf2_ros.Buffer`/`TransformListener` on `bl.shared_node`, polling up to 30s, with
  visible "Waiting..."/"available after N.Ns" log messages) between the `sim_slam` and
  `sim_nav2` includes. Note: must use `bl.shared_node`, not a node created via
  `rclpy.create_node()` — `better_launch` runs `rclpy.init()` against its own private
  `Context`, not the global default one, so a plain `rclpy.create_node()` call raises
  `NotInitializedException`. `bl.shared_node` is already spun continuously by
  `better_launch`'s own background executor thread, so the wait only needs to poll with
  `time.sleep()`, not call `spin_once()` itself.
- **`NO_FRONTIER_PATIENCE` raised 8→14 ticks (4s→7s)** — found via live testing after a
  spin: `slam_toolbox`'s `map_update_interval` (5.0s, never overridden) was *longer* than
  the old 4s patience window, so exploration could give up before `/map` had refreshed
  even once after the initial 360° spin (F13 T04q) revealed new area. 14 ticks gives one
  full 5s interval plus margin. 183/183 tests pass (tests reference the constant
  relatively, not hardcoded).
- **`inflation_radius` raised 0.16→0.17** (`config/nav2_param_patch.yaml`,
  `local_costmap`) — `controller_server` logged "the inflation radius (0.160000) is
  smaller than the circumscribed radius (0.164142)" on every run, forcing MPPI to do
  full-footprint collision checks on every candidate trajectory instead of using the
  costmap potential-field fast path. Measured actual robot speed at ~0.15 m/s against a
  configured 0.45 m/s cruise speed (via two Gazebo-native `/odom` position samples 2.25s
  apart) before the fix. This is shared config used by both sim and real-robot launches.
- **Still open**: with all of the above fixed, goals are now reliably *sent* but most
  still *fail* — one observed session showed `reached: 0, failed: 3, goal_num: 4`. This
  is now the primary blocker for F13 T05, not yet root-caused (candidates not yet
  investigated: read the actual `planner_server`/`controller_server`/`bt_navigator` logs
  for a specific failing goal). The "map isn't growing" symptom investigated at length
  this session (confirmed real via repeated `/map` occupancy-grid sampling, byte-for-byte
  identical over 25s at one point) turned out to be fully explained as a downstream
  consequence of this: `slam_toolbox` only folds a new pose into the map after 0.5m/0.5rad
  of net movement, and a robot that keeps failing goals rarely accumulates that much *net*
  progress even though it is genuinely moving frame-to-frame. Confirmed `slam_toolbox`
  itself is healthy — user's own manual teleop test made the map expand normally.

## Open issues (05-issues/open/)

**None open** as of 2026-07-08 — `05-issues/open/` is empty. I06–I09 are all in
`05-issues/closed/`. Verified 2026-07-08: I08 (test-file headers) all present; I06
(leading underscores) — its documented targets are clean, and two later-added helpers in
`frontier_explorer.py` (`_frontier_diag`, `_cell_out_of_range`) were renamed to drop the
underscore in this session's cleanup, so no leading-underscore violations remain in source.

## Likely next steps

1. **F13 T05** — sim exploration now works well (goals sent and reached, map fills in). The
   remaining known nav issues are diagnosed but not fully fixed and are candidates for a
   deliberate tuning pass / Nav2 discussion post: (a) ~0.11 m/s crawl on ~1 m goals =
   stock MPPI `GoalCritic.threshold_to_consider: 1.4`; (b) near-border "Start occupied"
   planner failures when the robot center is in an inflated/lethal cell; (c) planner choice
   (NavFn vs SmacPlanner2D — see TF13 T04p). None block basic exploration.
2. **F13 T06** — update feature/task file status, move to done.
3. **Architecture convergence — DONE (2026-07-08, TF10 T08).** `robot_explore.launch.py`
   now runs `pluggable_explore_manager_node` with real-robot params; the original
   `explore_manager_node.py` was deleted. Sim and real share one code path.
4. **Real-robot verification (open):** Modes A/B/E have never been live-run on hardware;
   the standalone `nav2_real.yaml`/`nav2_localization_real.yaml` are behavior-preserving
   copies but unproven (F10 T07).
4. **I06** — underscore rename sweep in remaining files
5. **I07, I08, I09** — verify/close quick wins
6. **`better_launch` fix upstreaming** — the `--param-file`→`--params-file` fix
   (2026-07-07) was applied directly to this workspace's local `src/better_launch`; if
   that repo tracks an upstream remote separately from this workspace, consider whether
   it should be contributed back.

## Exploration params (ExploreParams defaults + node ROS parameters)

**2026-07-08 changes to note first:** the initial 360° spin was **removed**;
`min_frontier_dist` is now a ROS parameter, **0.9 m in sim** / 1.3 m real (see below);
sim `yaw_goal_tolerance` raised to ~π (goals carry a fixed identity orientation, so a tight
yaw tolerance forced a wasteful end-of-goal spin); `explore_param_patch.yaml` no longer
exists — speed/costmap values now live directly in the standalone `nav2_explore_*.yaml`.
Note the `nav2_explore_*.yaml` header records that MPPI (the actual FollowPath plugin)
**ignores `desired_linear_vel`** — real cruise speed is governed by `vx_max`/`vx_min`.

- `desired_linear_vel`: historically 0.12 m/s real / 0.45 m/s sim, but this key is a no-op
  under MPPIController (see the `nav2_explore_*.yaml` audit note); effective speed is set by
  MPPI `vx_max` (0.45 sim).
- `MIN_FRONTIER_SIZE` / `min_frontier_size`: 10 cells at the `ExploreParams` dataclass
  level and the pluggable node's own ROS parameter default (real-robot value, unchanged).
  Sim launch files: briefly set to **1** on 2026-07-05 (F13 T04g) to test whether
  `prefer_farthest` could reach ~2m-class clusters — this **caused a 100% goal failure
  rate** (isolated single-cell frontier slivers at the ragged edge of known space broke
  NavFn's path reconstruction: `Failed to create a plan from potential when a legal
  potential was found`). Raised back to **5** the same day (F13 T04l) — still below the
  original 10, but excludes the pathological single-cell edge slivers.
- `MIN_FRONTIER_DIST` / `min_frontier_dist`: **1.3 m real, 0.9 m sim** (2026-07-08, TF13
  T04w). Now a ROS parameter on `pluggable_explore_manager_node` (default 1.3 = real value);
  sim launch files set 0.9 to fix the startup deadlock where the only large-enough frontier
  was < 1.3 m away so no goal was ever sent (`wed3` telemetry). `ExploreParams` default is
  1.3. (The original `explore_manager_node.py` referenced in older notes was deleted
  2026-07-08 — sim and real now share `pluggable_explore_manager_node`.)
- `MAX_FRONTIER_DIST`: 0.0 (unlimited) at the `ExploreParams` dataclass level; the pluggable
  sim node (`pluggable_explore_manager_node` / sim launch files) defaults its
  `max_frontier_dist` ROS parameter to **15.0 m** (raised from 3.0 on 2026-07-05, F13 T04o —
  3.0 was tuned for the smaller `simple_room.world` and silently capped `prefer_farthest`
  in the bigger `multi_room.world`, discarding genuinely reachable far frontiers; before
  that, raised from 1.0 on 2026-07-04 since 1.0 was below `MIN_FRONTIER_DIST`, an empty/
  impossible band that always failed to find frontiers — see F13 T04d)
- `prefer_farthest`: `False` at the `ExploreParams` dataclass level and on
  `pluggable_explore_manager_node`'s ROS parameter default; sim launch files default it to
  `True` (added 2026-07-04, F13 T04e) — selects the farthest qualifying frontier instead of
  nearest, to avoid repeatedly retrying wall-hugging frontiers near a failed obstacle.
  **`check_goal_redirect()` is now disabled whenever `prefer_farthest` is `True`** (F13 T04f,
  2026-07-05): mid-navigation redirect re-picks "best frontier" from the robot's *current*
  position every tick, which is unstable under farthest-first (the answer flips sides as
  soon as the robot moves toward it — confirmed via telemetry showing the robot ping-ponging
  between two points, 17 goals sent, 0 reached). Once a farthest-first goal is sent, the
  robot now commits to it instead of re-evaluating mid-flight.
- `BLACKLIST_RADIUS`: 0.5 m
- `GOAL_INSET_M`: 0.3 m (nudge goal off frontier boundary)
- `GOAL_TIMEOUT_S`: 25.0 s (break Nav2 BT recovery loops)
- `NO_FRONTIER_PATIENCE`: 14 ticks = 7 s at 2 Hz (raised from 8/4s on 2026-07-07 — must
  exceed slam_toolbox's `map_update_interval` of 5s, or patience can run out before
  `/map` has refreshed even once, e.g. right after the initial spin)
- `max_explore_radius`: 0.0 = unlimited

**`MIN_FRONTIER_DIST` 0.8→1.3 (2026-07-03)**: user request was "never ask Nav2 to go to a
point closer than a full meter away." `min_frontier_dist`/`MIN_FRONTIER_DIST` filters the
*raw frontier cell* distance, before `nudge_toward_robot()` pulls the actual Nav2 goal back
toward the robot by `GOAL_INSET_M` (0.3 m) — so the real floor on the sent goal is
`min_frontier_dist - GOAL_INSET_M`, not the raw value. `1.3 - 0.3 = 1.0` m gives the exact
requested floor. Changed in **both** `ExploreParams` (pluggable/sim node) and
`explore_manager_node.py`'s `MIN_FRONTIER_DIST` class constant (real robot) — the user
explicitly chose "both nodes" when asked, which is a deliberate, one-off exception to the
F12 rule that `explore_manager_node.py` stays untouched as the pluggable node's rollback-safe
original. See `01-literate/07-explore_manager_node.md`'s Observations section.

## Launch commands

```
# Base robot stack (ALWAYS first)
bl dome2 robot.launch.py --options "dri nav"

# Mode A: mapping
bl dome_nav robot_map.launch.py --map_name <name>

# Mode B: navigation
bl dome_nav robot_nav.launch.py --map_name <name>

# Mode E: autonomous exploration (original node)
bl dome_nav robot_explore.launch.py --map_name <name>
bl dome_nav robot_explore.launch.py --map_name <name> --max_explore_radius 8.0
```

## Sim launch commands (2026-07-07: two steps, Gazebo started separately)

`sim_robot.launch.py` (and therefore `sim_nav_full.launch.py`, which includes it) no
longer starts Gazebo itself — start it by hand first, then run the launch file:

```bash
# Step 1: start Gazebo (own terminal, wait for it to fully load)
gz sim -r ~/ros2_ws/install/dome_nav/share/dome_nav/worlds/multi_room.world

# Step 2: spawn the robot + full stack (second terminal, after Gazebo is up)
bl dome_nav sim_nav_full.launch.py --map_name <name> --world_name multi_room
```

## Exploration manual commands

```bash
# Start exploration
ros2 topic pub --once /intent std_msgs/msg/String \
  'data: "{\"name\": \"exploration_start\", \"source\": \"cli\", \"slots\": {}}"'

# Stop exploration
ros2 topic pub --once /intent std_msgs/msg/String \
  'data: "{\"name\": \"exploration_stop\", \"source\": \"cli\", \"slots\": {}}"'

# Watch status
ros2 topic echo /explore/status

# Watch telemetry
tail -f ~/.dome/telemetry/*.jsonl
```

## /explore/status JSON format

```json
{"state": "exploring", "reached": 3, "failed": 1, "goal_num": 5,
 "blacklisted": 2, "no_frontier_ticks": 0,
 "goal_xy": [1.23, 4.56], "dist_m": 1.87, "elapsed_s": 4.2}
```

Idle/done: `{"state": "idle", "reached": 0, "failed": 0}`

## /explore/markers MarkerArray

| namespace | type | color | content |
|---|---|---|---|
| `frontiers` (id=0) | POINTS | yellow | frontier cells from large clusters |
| `blacklist` (id=1) | POINTS | red | all blacklisted positions |
| `goal` (id=2) | SPHERE | cyan | current nav goal; DELETE when none |

## Intent contract

| dome_control command | intent name | slots |
|---|---|---|
| `nav go <label>` | `navigation_go` | `{"label": "<label>"}` |
| `nav cancel` | `navigation_cancel` | `{}` |
| `nav explore` | `exploration_start` | `{}` |
| `nav explore stop` | `exploration_stop` | `{}` |
