# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-25
**Branch:** main
**Status:** F10 live hardware testing in progress. Ring-cluster bug fixed,
telemetry added, goal timeout working. 85 dome_nav tests pass.
T06/T07 (live smoke test) still pending full completion.

## What exists

- `dome_nav/nav_manager.py` — pure Python `NavManager`: JSON parse (uses `"name"` key,
  label from `slots.label`), nearest-target, localization score, status strings
- `dome_nav/frontier_explorer.py` — pure Python frontier detection: OccupancyGrid scan,
  8-connectivity clustering, blacklist-aware nearest-cell selection (NOT centroid),
  max_radius and min_dist filters
- `dome_nav/explore_manager_node.py` — ROS2 node: `exploration_start`/`exploration_stop`
  intents → Nav2 NavigateToPose goals, blacklisting, 2 Hz timer loop, `/explore/status`
  (JSON), goal timeout (25s), telemetry via TelemetryWriter
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
  `test_frontier_explorer.py` (23, pure), `test_map_validation.py` (4, manual/live only)

## Test status

**85 passed, 4 deselected** (manual) via
`python3 -m pytest src/dome_nav/test/ -m "not manual"`.

## This session's work

### Live hardware testing of F10 (Mode E exploration)

Diagnosed and fixed multiple issues during live runs:

1. **No frontiers found immediately** — slam_toolbox loaded existing .posegraph file.
   Fix: use a fresh map_name each run.

2. **Ring-cluster bug** — large frontier ring surrounding robot has centroid ≈ robot
   position → filtered by MIN_FRONTIER_DIST. Fix: `pick_best_frontier` now returns
   nearest cell in cluster (not centroid). Blacklist checked per-cell, not per-cluster.
   Regression test added: `test_pick_ring_cluster_centroid_near_robot`.

3. **Goals too close** — GOAL_INSET_M was too large (1.0m), placing goals inside
   Nav2's xy_goal_tolerance (0.5m). Fixed: MIN_FRONTIER_DIST=2.0m, GOAL_INSET_M=0.3m.
   Nudged goal lands ~1.7m from robot.

4. **Long pauses between goals** — Nav2 BT running full recovery cycle (spin, retry)
   before reporting failure. Fix: 25s goal timeout in explore_manager_node. Cancels
   goal, blacklists centroid, picks next frontier immediately.

5. **`/explore/status` not publishing** — was only publishing on state transitions.
   Fix: publish at 2Hz in explore_tick. Now JSON with state, goal_num, goal_xy,
   dist_m, elapsed_s, blacklisted count.

6. **Telemetry** — `TelemetryWriter` extracted to `explore_telemetry.py`. Session
   file at `~/.dome/telemetry/<map_name>_<ts>.jsonl`. Events: session_start,
   goal_sent, goal_result (reached/failed/canceled/timeout), no_frontier, session_end.

7. **Style guide review** — leading underscores removed, debug cluster log removed,
   GOAL_STATUS_NAMES extracted as module constant, `check_goal_timeout` extracted,
   all lines ≤ 88 chars, telemetry in separate module.

8. **Method comments** — all ExploreManagerNode methods have comments explaining
   when they are called and why.

9. **Literate docs** — `07-explore_manager_node.md` updated (TD diagrams, colors,
   state machine updated). `06-frontier_explorer.md` updated (ring-cluster fix
   documented). `X05-explore_telemetry.md` created.

## Open issues (05-issues/open/)

- I06: leading-underscore MUST violations (3 source + 3 test files) — partially
  addressed in explore_manager_node; other files still pending
- I07: localization score not clamped to 1.0 → already clamped in current code; verify
  before closing
- I08: test files missing header
- I09: `should_save()` 1-line method — verify moot before closing

## Likely next steps

1. TF10 T06 — resolve open hardware questions: does explore auto-stop cleanly?
   speed cap tuning? narrow doorway behavior? (testing in progress)
2. TF10 T07 — full live smoke test: `nav explore` from dome_control CLI, complete run
3. I06 — underscore rename sweep in remaining files
4. I07, I08, I09 — verify/close quick wins
5. TF09 T04 — manual live smoke test: `nav go chair` from dome_control CLI

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
- `MIN_FRONTIER_SIZE`: 10 cells
- `MIN_FRONTIER_DIST`: 2.0 m (must exceed GOAL_INSET_M + xy_goal_tolerance)
- `BLACKLIST_RADIUS`: 0.5 m
- `GOAL_INSET_M`: 0.3 m (nudge goal off frontier boundary)
- `GOAL_TIMEOUT_S`: 25.0 s
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
{"state": "exploring", "goal_num": 3, "goal_xy": [1.23, 4.56], "dist_m": 1.87, "elapsed_s": 4.2, "blacklisted": 2}
```

Idle/done: `{"state": "idle"}` or `{"state": "done"}`

## AMCL notes (unchanged)

- Convergence: `covariance[0]` (x) and `covariance[7]` (y) both < 0.05 m² = converged
- `set_initial_pose: true` at basement1 dock pose (x=-2.768, y=0.145, yaw=1.743)
