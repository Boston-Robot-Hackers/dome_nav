# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-07-01
**Branch:** main
**Status:** F12 complete. F13 (Gazebo simulation) in progress: T01-T03 done, full sim stack
(Gazebo + slam_toolbox + Nav2 + explore) launches and drives the robot end-to-end. Two real
bugs found and fixed along the way (lidar sensor type, `collision_monitor` config); a
CPU-starvation performance issue is still being chased (see F13 status below). 152 pytest
tests pass (`pytest test/ -m "not manual"`). `tools/algo_demo.py` now has:
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
- `tools/nav_intent_check.py` — diagnostic: publishes target + intent, verifies nav pipeline

## Tests

| File | Count | Type |
|---|---|---|
| `test_nav_manager_pure.py` | 22 | pure Python |
| `test_utils_pure.py` | 5 | pure Python |
| `test_frontier_explorer.py` | 31 | pure Python |
| `test_frontier_algorithm.py` | 11 | pure Python |
| `test_nav_manager.py` | 18 | ROS mock |
| `test_slam_manager.py` | 11 | ROS lifecycle |
| `test_explore_manager_node.py` | 30 | ROS mock |
| `test_pluggable_explore_manager_node.py` | ~22 | ROS mock |
| `test_map_validation.py` | 4 | manual/live only |

**74 pure-Python tests pass** (`test_frontier_algorithm.py` + `test_frontier_explorer.py` + `test_nav_manager_pure.py` + `test_utils_pure.py`).
ROS mock tests require ROS2 environment (run on robot).

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
  `gz_args` so `gz sim` runs server-only). Added so RViz2 can be used instead of the
  Gazebo GUI, and because the software-rendered GUI (`LIBGL_ALWAYS_SOFTWARE=1`) consumes
  CPU that Nav2's control loop needs (see T04 below) — not yet confirmed whether headless
  actually fixes the slow-motion issue.
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
- T04 (sim_time propagation) — **in progress, found a real performance problem**: telemetry
  from a full exploration run (`~/.dome/telemetry/explore-newtest4-*.jsonl`) showed a goal
  only 0.5 m away hit the 25s `GOAL_TIMEOUT_S` and the robot moved only 4 cm in that time —
  even after the sim speed override above. `controller_server` logs show
  `Control loop missed its desired rate of 20.0000 Hz. Current loop rate is 15-45 Hz`
  concurrently. Root cause hypothesis: **CPU starvation**, not a config or sim_time bug —
  this VM has only 2 cores (`nproc` = 2) running Gazebo physics + software-rendered GUI +
  `ros_gz_bridge` + slam_toolbox (Ceres solver) + full 12-node Nav2 stack (including MPPI,
  which is itself compute-heavy) simultaneously. Under that load MPPI likely can't build
  confident fast trajectories per cycle and falls back to near-crawl velocities. Next step:
  retest with `--headless true` (removes GUI rendering load) and/or lower
  `controller_server`'s target frequency as a sim-only tweak, then compare telemetry for
  the same goal distance.
- **Unexplained discrepancy (not yet investigated)**: frontier goals selected during
  exploration are consistently ~0.5 m from the robot even though `MIN_FRONTIER_DIST` is
  configured at 0.8 m — worth checking `pick_best_frontier`/`nudge_toward_robot` behavior
  in this specific map, since goal ends up equal to frontier_xy (no inset applied) in the
  telemetry.
- T05 (end-to-end exploration smoke test), T06 (docs/move to done) — blocked on resolving
  the T04 performance issue first; not yet passable given the crawl-speed behavior above.

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

Also note: `setup.py` was updated to add the `pluggable_explore_manager_node` console entry
point and to install `worlds/*` as package share data — both required for T03 and already
in place.

## Open issues (05-issues/open/)

- I06: leading-underscore MUST violations (3 source + 3 test files) — partially addressed
- I07: localization score not clamped to 1.0 → already clamped; verify before closing
- I08: test files missing header
- I09: `should_save()` 1-line method — verify moot before closing

## Likely next steps

1. **F13 T04** — retest with `--headless true` to see if removing GUI rendering fixes the
   CPU-starvation crawl-speed issue; if not, try lowering `controller_server` frequency
   as a sim-only tweak. Also verify TF/costmap/Nav2 timestamps are on sim clock as originally scoped.
2. **F13** — investigate the ~0.5m frontier goal distance vs. configured `MIN_FRONTIER_DIST: 0.8`
3. **F13 T05** — end-to-end exploration smoke test in sim (blocked until T04's speed issue resolved)
4. **F13 T06** — update feature/task file status, move to done, update this doc
5. **I06** — underscore rename sweep in remaining files
6. **I07, I08, I09** — verify/close quick wins
7. **TF10 T06** — hop-size issue: increase `MIN_FRONTIER_DIST` 0.8→1.5m or add
   cluster-size preference to `pick_best_frontier`

## Exploration params (explore_param_patch.yaml)

- `desired_linear_vel`: 0.12 m/s
- `MIN_FRONTIER_SIZE`: 10 cells (noise threshold; good range 5–20)
- `MIN_FRONTIER_DIST`: 0.8 m (must exceed GOAL_INSET_M + xy_goal_tolerance = 0.55 m)
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
