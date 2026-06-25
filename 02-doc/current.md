# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-24
**Branch:** main
**Status:** F10 T01–T05 implemented. Autonomous frontier exploration working on
hardware (first live run completed). 84 dome_nav + 202 dome_control tests pass.
T06 (open questions) and T07 (full live smoke test) pending.

## What exists

- `dome_nav/nav_manager.py` — pure Python `NavManager`: JSON parse (uses `"name"` key,
  label from `slots.label`), nearest-target, localization score, status strings
- `dome_nav/frontier_explorer.py` — pure Python frontier detection: OccupancyGrid scan,
  8-connectivity clustering, blacklist-aware nearest-centroid selection, max_radius
  and min_dist filters
- `dome_nav/explore_manager_node.py` — ROS2 node: `exploration_start`/`exploration_stop`
  intents → Nav2 NavigateToPose goals, blacklisting, 2 Hz timer loop, `/explore/status`
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
  `test_frontier_explorer.py` (22, pure), `test_map_validation.py` (4, manual/live only)

## Test status

**84 passed, 4 deselected** (manual) via
`python3 -m pytest src/dome_nav/test/ -m "not manual"`.

## This session's work

- Deleted `how_to_be.md` from dome_nav and j3; removed all references
- Renamed all intents to noun-verb: `navigation_go`, `navigation_cancel`,
  `exploration_start`, `exploration_stop` (dome_nav + dome_control, 260 pass)
- Added MUST rule to style_guide.md (dome_nav + j3): `bl` CLI syntax for launch files
- Added F10 + TF10: autonomous frontier exploration
- Implemented F10 T02–T05:
  - `frontier_explorer.py` — pure Python frontier detection + clustering + filters
  - `explore_manager_node.py` — ROS2 node with intent routing, blacklisting, nudge inset
  - `explore_param_patch.yaml` — slow speed (0.12 m/s) for slam_toolbox stability
  - `robot_explore.launch.py` — Mode E launch
  - dome_control: `nav.explore` / `nav.explore.stop` commands
- Added `max_explore_radius` feature: limits map to a circle from start position
- Fixed two live hardware bugs:
  - Frontier loop (same goal repeated): blacklist on success + MIN_FRONTIER_DIST filter
  - worldToMap boundary error: nudge goal 0.3 m inward toward robot
- Updated literate docs: 03, 05 (intent rename); new 06, 07

## Open issues (05-issues/open/)

- I06: leading-underscore MUST violations (3 source + 3 test files)
- I07: localization score not clamped to 1.0 → already clamped in current code; verify
  before closing
- I08: test files missing header
- I09: `should_save()` 1-line method — verify moot before closing

## This session's work

- Fixed dispatch_text 3-token bug: `nav explore stop` now routes to `nav.explore.stop`
  (not `nav.explore`). Dispatcher tries `command.second.third` before `command.second`.
- Added `nav.explore.status` command: reads `/explore/status` topic via
  `ros2 topic echo --once`, does NOT publish any intent.
- `explore_status()` added to robot_controller.py; `subprocess` + 3s timeout.
- 3 new tests; 199 → 202 dome_control tests pass.
- Regenerated literate docs: 11-robot_controller.md, 12-command_dispatcher.md, X04-navigation_commands.md.

## Likely next steps

1. TF10 T06 — resolve open questions on hardware: does explore auto-stop cleanly?
   speed cap tuning? narrow doorway behavior?
2. TF10 T07 — full live smoke test: `nav explore` from dome_control, complete run
3. I06 — underscore rename sweep
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

## Exploration params (explore_param_patch.yaml)

- `desired_linear_vel`: 0.12 m/s
- `max_velocity`: [0.15, 0.0, 1.0]
- `MIN_FRONTIER_SIZE`: 10 cells
- `MIN_FRONTIER_DIST`: 0.5 m from robot
- `BLACKLIST_RADIUS`: 0.5 m
- `GOAL_INSET_M`: 0.3 m (nudge toward robot)
- `max_explore_radius`: 0.0 = unlimited (pass via `--max_explore_radius <m>`)

## Launch commands

```
bl robot_map.launch.py --map_name <name>           # Mode A: mapping
bl robot_nav.launch.py --map_name <name>           # Mode B: navigation
bl robot_explore.launch.py --map_name <name>       # Mode E: autonomous exploration
bl robot_explore.launch.py --map_name <name> --max_explore_radius 8.0
```

## AMCL notes (unchanged)

- Convergence: `covariance[0]` (x) and `covariance[7]` (y) both < 0.05 m² = converged
- `set_initial_pose: true` at basement1 dock pose (x=-2.768, y=0.145, yaw=1.743)
