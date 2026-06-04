# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-04
**Branch:** main
**Status:** bootstrapped — scaffold only, no tests yet.

## What exists

- Package scaffolded: `package.xml`, `setup.py`, `CLAUDE.md`, `.claude/`.
- `slam_manager_node.py` — monitors `/map`, saves pose graph on shutdown.
- `nav_manager_node.py` — subscribes `/intent`, sends `NavigateToPose` goals to Nav2.
- `config/slam.yaml` — slam_toolbox params (from linorobot2, with map persistence enabled).
- `config/nav2.yaml` — Nav2 params (from linorobot2, AMCL removed).
- `launch/robot.launch.py` and `launch/remote.launch.py` — better_launch wrappers.

## What is NOT done

- No features or tasks filed yet — needs F01 and TF01 before any code changes.
- Package not built or tested with colcon.
- dome_vision `semantic_map_node.py` still uses `odom` frame — needs updating to `map`.
- No integration test with slam_toolbox running.
- `nav_manager_node.py` goal frame uses `map` — requires slam_toolbox to be running.

## Likely Next Steps

1. File F01 (slam bring-up and map persistence) with tasks.
2. File F02 (intent-driven navigation to confirmed targets) with tasks.
3. Update `dome_vision` `semantic_map_node.py` TF target frame: `odom` → `map`.
4. Build and smoke-test with `colcon build --packages-select dome_nav`.
5. Integration test: run linorobot2 + dome_nav, verify `map` frame appears.
