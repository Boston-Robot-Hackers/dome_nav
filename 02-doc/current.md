# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-04
**Branch:** main
**Status:** bootstrapped — scaffold only, no feature/task files yet.

## What exists

- Package scaffolded: `package.xml`, `setup.py`, `CLAUDE.md`, `.claude/`.
- `slam_manager_node.py` — monitors `/map`, saves pose graph on shutdown.
- `nav_manager_node.py` — subscribes `/intent`, sends `NavigateToPose` goals to Nav2.
- `utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()` for launch config merging.
- `config/slam.yaml` — slam_toolbox params (from linorobot2, with map persistence enabled).
- `config/nav2.yaml` — Nav2 params (from linorobot2, AMCL removed).
- `launch/robot.launch.py` and `launch/remote.launch.py` — better_launch wrappers.
- `01-literate/` — literate docs for utils, slam_manager_node, nav_manager_node.

## Code Issues Found (not yet filed as tasks)

- `nav_manager_node.py:75` — `self.nav_client._cancel_goal_async()` uses private rclpy API; should track GoalHandle and call `goal_handle.cancel_goal_async()`.
- `nav_manager_node.py:82` — `find_nearest_confirmed` returns first match, not nearest; name is misleading.
- `slam_manager_node.py:45` — `os.path.dirname` returns `""` if path has no directory; `makedirs("")` raises. Guard needed.
- `slam_manager_node.py` — `save_map` uses `spin_until_future_complete` at shutdown; if slam_toolbox already exited, blocks 10s.

## What is NOT done

- No features or tasks filed yet — needs F01 and TF01 before any code changes.
- Package not built or tested with colcon.
- dome_vision `semantic_map_node.py` still uses `odom` frame — needs updating to `map`.
- No integration test with slam_toolbox running.
- No unit tests; test/ directory is empty.

## Likely Next Steps

1. File F01 (slam bring-up and map persistence) with tasks.
2. File F02 (intent-driven navigation to confirmed targets) with tasks.
3. Fix `find_nearest_confirmed` — rename or implement distance sort.
4. Fix `cancel_navigation` to use tracked GoalHandle instead of private API.
5. Build and smoke-test with `colcon build --packages-select dome_nav`.
6. Integration test: run linorobot2 + dome_nav, verify `map` frame appears.
