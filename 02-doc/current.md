# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-07
**Branch:** main
**Status:** F01 complete — all T01–T06 done. slam_toolbox launches, /map publishes, TF valid, status publishes, pose graph saves periodically, map loads on relaunch.

## What exists

- Package scaffolded: `package.xml`, `setup.py`, `CLAUDE.md`, `.claude/`.
- `slam_manager_node.py` — monitors `/map`, publishes `/dome_nav/slam_status`, saves pose graph every 30s and on shutdown.
- `nav_manager_node.py` — subscribes `/intent`, sends `NavigateToPose` goals to Nav2.
- `utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()` for launch config merging.
- `config/slam_param_patch.yaml` — slam_toolbox overrides (range, TF tolerance, map persistence).
- `config/nav2_param_patch.yaml` — Nav2 overrides (AMCL removed).
- `launch/robot.launch.py` and `launch/remote.launch.py` — better_launch wrappers.
- `01-literate/` — literate docs for utils, slam_manager_node, nav_manager_node.
- `test/test_map_validation.py` — manual integration tests for T02 (4 tests, all pass).
- `test/test_slam_manager.py` — unit tests for T06: on_map and save_map (7 tests, all pass).
- `setup.cfg` — pytest `manual` mark registered.

## F01/TF01 Task Status

- T01 — done (colcon build passes)
- T02 — done (slam_toolbox launches, /map and map→odom TF confirmed, 4 tests pass)
- T03 — done (`/dome_nav/slam_status` publishes "mapping")
- T04 — done (pose graph saves every 30s via periodic timer)
- T05 — done (map loads on next run — 69 occupied cells visible immediately on relaunch)
- T06 — done (7 unit tests for slam_manager_node — save_map mocked, on_map status)

## Key Design Decision — T04

Ctrl-C sends SIGINT to all nodes simultaneously. slam_toolbox exits at same time as
slam_manager, so the `finally: save_map()` call in main() usually fails (service gone).
Fix: `periodic_save` timer fires every 30s while map_ready, ensuring pose graph is saved
regardless of shutdown ordering. Shutdown save is best-effort fallback only.

## Code Issues Fixed

- `nav_manager_node.py` — `cancel_navigation` now tracks GoalHandle via `_on_goal_accepted` callback, calls `goal_handle.cancel_goal_async()`.
- `nav_manager_node.py` — `find_nearest_confirmed` now does true distance sort using TF; falls back to first match if TF unavailable.
- `slam_manager_node.py` — `makedirs("")` guard added: only calls `makedirs` when `dirname` is non-empty.

## What is NOT done

- F02+ features not yet defined.
- dome_vision `semantic_map_node.py` still uses `odom` frame — needs updating to `map`.
- No integration test with full linorobot2 + dome_nav stack automated.

## F02/TF02 Task Status

- T01 — done (find_nearest_confirmed: distance sort, 6 tests)
- T02 — done (cancel_navigation: tracked GoalHandle via _on_goal_accepted)
- T03 — done (_on_goal_result: publishes done:/failed: on completion, 4 tests)
- T04 — done (18 unit tests total: routing, navigate, cancel, result callbacks)
- T05 — not done (manual integration test — needs live stack)

## T05 Live Stack Debugging (2026-06-07)

F02 T05 manual test in progress. Status flow works: `navigating:can` → `failed:can` confirmed.
Failure cause: Nav2 stack not fully up.

**Findings:**
- `/targets/confirmed` had 0 publishers — `semantic_map` lifecycle node was `unconfigured`
- Fixed: `ros2 lifecycle set /semantic_map configure && ros2 lifecycle set /semantic_map activate`
- After activation, Nav2 accepted goal but `compute_path_to_pose` action server timed out
- `bt_navigator` running but planner server not ready or not started
- `collision_monitor` missing params (`FootprintApproach.max_points`)
- `opennav_docking` missing param (`dock_database`)
- `dome_control` not running — hardware interface absent

**Next debug step:**
```bash
ros2 node list | grep -E "planner|controller|costmap"
ros2 action list | grep compute_path
```

## Likely Next Steps

1. T05: get full Nav2 stack (planner + controller) running, retry intent navigation.
2. Fix dome_vision `semantic_map_node.py` frame from `odom` → `map`.
3. Define F03.
