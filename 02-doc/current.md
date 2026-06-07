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

## Code Issues Found (not yet filed as tasks)

- `nav_manager_node.py:75` — `self.nav_client._cancel_goal_async()` uses private rclpy API; should track GoalHandle and call `goal_handle.cancel_goal_async()`.
- `nav_manager_node.py:82` — `find_nearest_confirmed` returns first match, not nearest; name is misleading.
- `slam_manager_node.py:50` — `os.path.dirname` returns `""` if path has no directory; `makedirs("")` raises. Guard needed.

## What is NOT done

- F02+ features not yet defined.
- dome_vision `semantic_map_node.py` still uses `odom` frame — needs updating to `map`.
- No integration test with full linorobot2 + dome_nav stack automated.

## Likely Next Steps

1. Define F02 feature (nav_manager_node — NavigateToPose flow).
2. Fix `find_nearest_confirmed` — rename or implement distance sort.
3. Fix `cancel_navigation` to use tracked GoalHandle instead of private API.
4. Fix `makedirs("")` guard in `save_map`.
