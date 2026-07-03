# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-07-03
**Branch:** main
**Status:** F12 complete. F13 (Gazebo simulation) in progress: T01-T03 done, full sim stack
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
debugging (now consolidated to 4: `sim_robot.launch.py`, `sim_nav.launch.py`,
`sim_rviz.launch.py`, `sim_explore_node.launch.py` — see F13 status below). This live,
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
- `dome_nav/explore_manager_node.py` — **original, untouched** ROS2 node: all F12 work
  is additive; this file reverts to before F12 if needed
- `dome_nav/explore_context.py` — **(F12 new)** `ExploreParams`, `ExplorationContext`
  dataclasses and `ExplorationAlgorithm` Protocol
- `dome_nav/frontier_algorithm.py` — **(F12 new)** `FrontierAlgorithm` class wrapping
  the pure frontier functions behind the protocol
- `dome_nav/explore_markers.py` — **(F12 new)** pure functions for RViz `MarkerArray`
  construction (frontiers/blacklist/goal markers); extracted for node file-length budget
- `dome_nav/pluggable_explore_manager_node.py` — **(F12 new)** copy-and-modify of
  `explore_manager_node.py` that accepts injected `ExplorationAlgorithm`; adds
  mid-navigation re-evaluation via `check_goal_redirect()` + `is_redirecting` flag:
  cancels current goal without blacklisting if best frontier shifts >1.5 m (`REDIRECT_THRESHOLD`)
- `dome_nav/explore_telemetry.py` — JSONL session logger
- `dome_nav/slam_manager_node.py` — **LifecycleNode**: watches `/map`, saves pose graph
  on first map receipt + every 30s
- `dome_nav/nav_manager_node.py` — ROS2 node: `/intent` → NavigateToPose, status,
  `/amcl_pose` → localization status/score
- `dome_nav/utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()`,
  `write_config()`
- `tools/algo_demo.py` — **(F12 new)** interactive CLI demo of `FrontierAlgorithm` on
  hand-crafted ASCII maps. ANSI 256-color; shows clusters A-Z, target T, goal G, robot R,
  blacklist B. Maps: `room`, `corridor`, `ring`, `maze`, `large` (30×30, 3-room layout).
  Now simulates lidar scanning along travel path via `uncover_along_path()` (sweeps at
  radius/2 steps from old to new robot position). Args: `--map`, `--inset`, `--min-size`,
  `--min-dist`, `--sensor-radius`, `--auto`
- `config/` — slam_param_patch, nav2_param_patch, nav2_amcl_patch, explore_param_patch
- `launch/robot_map.launch.py` (Mode A), `robot_nav.launch.py` (Mode B),
  `robot_explore.launch.py` (Mode E)
- `launch/sim_explore.launch.py` — **(F13)** full sim stack in one file (Gazebo, bridge,
  RSP, laser TF, slam_toolbox, Nav2, `slam_manager_node`, `pluggable_explore_manager_node`)
- `launch/sim_robot.launch.py`, `sim_nav.launch.py`, `sim_rviz.launch.py`,
  `sim_explore_node.launch.py` — **(F13, 2026-07-03)** the same sim stack split into 4
  single-purpose files for manual, one-window-per-piece debugging — see F13 status below
- `tools/nav_intent_check.py` — diagnostic: publishes target + intent, verifies nav pipeline

## Tests

| File | Count | Type |
|---|---|---|
| `test_nav_manager_pure.py` | 27 | pure Python |
| `test_utils_pure.py` | 5 | pure Python |
| `test_frontier_explorer.py` | 34 | pure Python |
| `test_frontier_algorithm.py` | 11 | pure Python |
| `test_nav_manager.py` | 19 | ROS mock |
| `test_slam_manager.py` | 11 | ROS lifecycle |
| `test_explore_manager_node.py` | 24 | ROS mock |
| `test_pluggable_explore_manager_node.py` | 24 | ROS mock |
| `test_map_validation.py` | 4 | manual/live only |

**77 pure-Python tests pass** (`test_frontier_algorithm.py` + `test_frontier_explorer.py` + `test_nav_manager_pure.py` + `test_utils_pure.py`).
**155/159 total pass** via `pytest test/ -m "not manual"` (4 deselected are `test_map_validation.py`'s manual/live-only tests).

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

## Open issues (05-issues/open/)

- I06: leading-underscore MUST violations (3 source + 3 test files) — partially addressed
- I07: localization score not clamped to 1.0 → already clamped; verify before closing
- I08: test files missing header
- I09: `should_save()` 1-line method — verify moot before closing

## Likely next steps

1. **F13 T04** — fix the doorway costmap-inflation stall (see T04 finding #5b above): reduce
   `local_costmap.inflation_layer.inflation_radius` (currently 0.2 m) and/or widen the
   doorway in `worlds/simple_room.world` (currently 0.6 m) so the robot has genuine low-cost
   clearance to cross it; the `max_frontier_dist` cap alone did not resolve this.
2. **F13** — reconsider whether `max_frontier_dist`'s operational default of 1.0 m is right
   for `simple_room.world` given the "no frontiers found" issue found 2026-07-03 — a single
   scan reveals most of the 4x4 m room immediately, likely pushing the nearest real frontier
   outside the 0.8–1.0 m band. Try a larger default (e.g. 2–3 m) or make it adaptive.
3. **F13** — the TF-extrapolation/collision_monitor stop is currently believed to be a
   process-hygiene side effect (stale/duplicate `/clock` source), not a structural bug — keep
   an eye out if it recurs in a verified-clean run.
4. **F13 T05** — end-to-end exploration smoke test in sim (blocked until the doorway stall is resolved)
5. **F13 T06** — update feature/task file status, move to done, update this doc
6. **I06** — underscore rename sweep in remaining files
7. **I07, I08, I09** — verify/close quick wins
8. **TF10 T06** — hop-size issue: increase `MIN_FRONTIER_DIST` 0.8→1.5m or add
   cluster-size preference to `pick_best_frontier`

## Exploration params (explore_param_patch.yaml + ExploreParams defaults)

- `desired_linear_vel`: 0.12 m/s
- `MIN_FRONTIER_SIZE`: 10 cells (noise threshold; good range 5–20)
- `MIN_FRONTIER_DIST`: 0.8 m (must exceed GOAL_INSET_M + xy_goal_tolerance = 0.55 m)
- `MAX_FRONTIER_DIST`: 0.0 (unlimited) at the `ExploreParams` dataclass level; the pluggable
  sim node (`pluggable_explore_manager_node` / `sim_explore.launch.py`) defaults its
  `max_frontier_dist` ROS parameter to 1.0 m, capping exploration hops in sim
- `BLACKLIST_RADIUS`: 0.5 m
- `GOAL_INSET_M`: 0.3 m (nudge goal off frontier boundary)
- `GOAL_TIMEOUT_S`: 25.0 s (break Nav2 BT recovery loops)
- `NO_FRONTIER_PATIENCE`: 8 ticks = 4 s at 2 Hz
- `max_explore_radius`: 0.0 = unlimited

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
