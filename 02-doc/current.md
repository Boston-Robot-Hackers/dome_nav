# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-27
**Branch:** main
**Status:** F12 complete + mid-navigation redirect implemented. 42 pure-Python tests pass.
`tools/algo_demo.py` demo now uncovers cells along the robot's travel path (not just at
destination). `pluggable_explore_manager_node.py` re-evaluates the frontier every tick
during transit and cancels/redirects if the best goal has shifted >1.5 m.

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

**42 pure-Python tests pass** (`test_frontier_algorithm.py` + `test_frontier_explorer.py`).
ROS mock tests require ROS2 environment (run on robot).

## F12 summary (this session)

All tasks done. Architecture is additive — original files untouched.

- **T01** `explore_context.py` — `ExploreParams`, `ExplorationContext`, `ExplorationAlgorithm` Protocol
- **T02** `frontier_algorithm.py` — `FrontierAlgorithm` wraps pure frontier functions
- **T03** `pluggable_explore_manager_node.py` — pluggable ROS2 node (copy-and-modify)
- **T04** `test_frontier_algorithm.py` — 11 pure Python tests (all passing)
- **T05** `test_pluggable_explore_manager_node.py` — ROS mock tests (syntax verified)
- **T06** Full suite regression — 42/42 pure tests pass
- **T07** `tools/algo_demo.py` — interactive CLI demo with color, clusters, 5 maps

**Post-F12 enhancements (same session):**
- `algo_demo.py`: `uncover_along_path()` — sweeps lidar reveal along travel path, not
  just at destination. Step size = sensor_radius/2 ensures full coverage.
- `pluggable_explore_manager_node.py`: `check_goal_redirect()` — mid-navigation
  re-evaluation every tick. Cancels current goal (without blacklisting, via
  `is_redirecting` flag) if best frontier shifts >1.5 m (`REDIRECT_THRESHOLD`).
  Telemetry event: `"redirect"` with old/new goal xy and shift distance.

**Future directions noted in `02-doc/notes.md`**:
- `CostmapFrontierAlgorithm` using `/global_costmap/costmap` instead of raw `/map`
- Adaptive goal distance (prefer far frontiers when near ones are on the travel path)
- Directional continuity bonus (discount frontiers already covered by path scanning)

## Open issues (05-issues/open/)

- I06: leading-underscore MUST violations (3 source + 3 test files) — partially addressed
- I07: localization score not clamped to 1.0 → already clamped; verify before closing
- I08: test files missing header
- I09: `should_save()` 1-line method — verify moot before closing

## Likely next steps

1. **I06** — underscore rename sweep in remaining files
2. **I07, I08, I09** — verify/close quick wins
3. **TF10 T06** — hop-size issue: increase `MIN_FRONTIER_DIST` 0.8→1.5m or add
   cluster-size preference to `pick_best_frontier`
4. **TF10 T07** — live smoke test after hop-size tuning
5. **Live test** `pluggable_explore_manager_node` on hardware (swap in via launch file)

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
