# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-26
**Branch:** main
**Status:** F11 (RViz markers) implemented and verified live. 117 tests pass.
Exploration working on hardware — robot moving, goals reaching. MIN_FRONTIER_DIST
hop-size issue identified.

## What exists

- `dome_nav/nav_manager.py` — pure Python `NavManager`: JSON parse (uses `"name"` key,
  label from `slots.label`), nearest-target, localization score, status strings
- `dome_nav/frontier_explorer.py` — pure Python frontier detection: OccupancyGrid scan,
  8-connectivity clustering, blacklist-aware nearest-cell selection (NOT centroid),
  max_radius and min_dist filters, `nudge_toward_robot` geometry helper
- `dome_nav/explore_manager_node.py` — ROS2 node: `exploration_start`/`exploration_stop`
  intents → Nav2 NavigateToPose goals, blacklisting, 2 Hz timer loop, `/explore/status`
  (JSON), `/explore/markers` (MarkerArray), goal timeout (25s), telemetry via
  TelemetryWriter. Key methods: `reset_session()`, `clear_active_goal()`,
  `find_and_send_frontier()`, `check_goal_timeout()`, `stop_exploring()`,
  `publish_status()`, `publish_markers()`
- `dome_nav/explore_telemetry.py` — JSONL session logger: one file per session in
  `~/.dome/telemetry/<map_name>_<ts>.jsonl`
- `dome_nav/slam_manager_node.py` — **LifecycleNode**: watches `/map`, saves pose graph
  on first map receipt + every 30s.
- `dome_nav/nav_manager_node.py` — ROS2 node: `/intent` → NavigateToPose, status,
  `/amcl_pose` → localization status/score.
- `dome_nav/utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()`,
  `write_config()`
- `config/` — slam_param_patch, nav2_param_patch, nav2_amcl_patch, explore_param_patch
- `launch/robot_map.launch.py` (Mode A) — accepts `map_name` arg (required, no default)
- `launch/robot_nav.launch.py` (Mode B) — AMCL + Nav2
- `launch/robot_explore.launch.py` (Mode E) — Mode A + explore_manager_node; accepts
  `map_name` (required) and `max_explore_radius` (default 0.0 = unlimited)
- `tools/nav_intent_check.py` — diagnostic: publishes target + intent, verifies nav pipeline
- Tests: `test_nav_manager_pure.py` (22), `test_utils_pure.py` (5),
  `test_nav_manager.py` (18, ROS), `test_slam_manager.py` (11, ROS lifecycle),
  `test_frontier_explorer.py` (31, pure), `test_explore_manager_node.py` (30, ROS mock),
  `test_map_validation.py` (4, manual/live only)

## Test status

**117 passed, 4 deselected** (manual) via
`python3 -m pytest src/dome_nav/test/ -m "not manual"`.

## This session's work

### F11 — RViz2 Exploration Markers (T01–T05 complete)

1. **T01**: `package.xml` — added `<depend>visualization_msgs</depend>`
2. **T02**: `explore_manager_node.py` — added `marker_pub`, `latest_clusters`,
   `latest_map_info` state; imported `Point`, `Marker`, `MarkerArray`, `cell_to_world`
3. **T03**: `find_and_send_frontier` — stores `self.latest_clusters` and
   `self.latest_map_info` each tick
4. **T04**: `publish_markers()` — three namespaces: `frontiers` (yellow POINTS),
   `blacklist` (red POINTS), `goal` (cyan SPHERE). DELETE markers when not exploring.
5. **T05**: `explore_tick()` — calls `publish_markers()` alongside `publish_status()`
6. **T06**: Manual RViz2 smoke test — **verified live**. Yellow frontier starburst,
   cyan goal sphere visible. Markers working correctly.

### Live test observations (2026-06-26, test_run4 telemetry)

Exploration IS working — robot moving and reaching goals:
- Goals 2 and 4 reached in 2–3s with ~0.3m actual robot movement
- Goals 1 and 3 timed out (25s) — Nav2 BT recovery loops, then blacklisted + skipped

**Key finding — hop size issue:**
All frontier picks land at exactly MIN_FRONTIER_DIST (0.8m) from robot. After
GOAL_INSET_M=0.3m nudge, goal is always ~0.5m away. Robot makes tiny hops.

From telemetry: frontier_xy distances from robot are all ~0.80m (1–4 goals).
`pick_best_frontier` returns the nearest frontier cell → always at the threshold.

**Two fixes under consideration:**
1. Increase `MIN_FRONTIER_DIST` 0.8→1.5m — force larger hops, faster coverage
2. Change `pick_best_frontier` to prefer large clusters over nearest cell —
   avoids tiny wall-edge clusters close by, prefers open-area clusters further away

## Open issues (05-issues/open/)

- I06: leading-underscore MUST violations (3 source + 3 test files) — partially
  addressed in explore_manager_node; other files still pending
- I07: localization score not clamped to 1.0 → already clamped in current code; verify
  before closing
- I08: test files missing header
- I09: `should_save()` 1-line method — verify moot before closing

## Likely next steps

1. **TF10 T06** — resolve hop-size issue: pick between MIN_FRONTIER_DIST increase
   vs. cluster-size-preference strategy in `pick_best_frontier`
2. **TF10 T07** — full live smoke test after tuning
3. **TF11 T06** — already done (verified live this session)
4. **TF09 T04** — live smoke test `nav go chair`
5. **I06** — underscore rename sweep in remaining files
6. **I07, I08, I09** — verify/close quick wins

## Intent contract

All intents: `{"name": <intent>, "source": "cli", "slots": {...}}`

| dome_control command | intent name | slots |
|---|---|---|
| `nav go <label>` | `navigation_go` | `{"label": "<label>"}` |
| `nav cancel` | `navigation_cancel` | `{}` |
| `nav explore` | `exploration_start` | `{}` |
| `nav explore stop` | `exploration_stop` | `{}` |

## Exploration params (explore_param_patch.yaml + explore_manager_node.py)

- `desired_linear_vel`: 0.12 m/s
- `max_velocity`: [0.15, 0.0, 1.0]
- `deadband_velocity`: [0.05, 0.0, 0.1]
- `MIN_FRONTIER_SIZE`: 10 cells (noise threshold; good range 5–20)
- `MIN_FRONTIER_DIST`: 0.8 m (must exceed GOAL_INSET_M + xy_goal_tolerance = 0.55 m)
- `BLACKLIST_RADIUS`: 0.5 m (covers centroid drift across map updates)
- `GOAL_INSET_M`: 0.3 m (nudge goal off frontier boundary)
- `GOAL_TIMEOUT_S`: 25.0 s (break Nav2 BT recovery loops)
- `NO_FRONTIER_PATIENCE`: 8 ticks = 4 s at 2 Hz
- `max_explore_radius`: 0.0 = unlimited (pass via `--max_explore_radius <m>`)

## Launch commands

```
# Base robot stack (ALWAYS first)
bl dome2 robot.launch.py --options "dri nav"

# Mode A: mapping
bl dome_nav robot_map.launch.py --map_name <name>

# Mode B: navigation
bl dome_nav robot_nav.launch.py --map_name <name>

# Mode E: autonomous exploration
bl dome_nav robot_explore.launch.py --map_name <name>
bl dome_nav robot_explore.launch.py --map_name <name> --max_explore_radius 8.0
```

## Exploration manual commands (dome_control CLI broken)

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

Idle/done: `{"state": "idle", "reached": 0, "failed": 0}` or
`{"state": "done", "reached": 5, "failed": 1}`

`goal_xy`, `dist_m`, `elapsed_s` omitted when no active goal or TF unavailable.

## /explore/markers MarkerArray

| namespace | type | color | content |
|---|---|---|---|
| `frontiers` (id=0) | POINTS | yellow | frontier cells from large clusters |
| `blacklist` (id=1) | POINTS | red | all blacklisted positions |
| `goal` (id=2) | SPHERE | cyan | current nav goal; DELETE when none |

## AMCL notes (unchanged)

- Convergence: `covariance[0]` (x) and `covariance[7]` (y) both < 0.05 m² = converged
- `set_initial_pose: true` at basement1 dock pose (x=-2.768, y=0.145, yaw=1.743)
