# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-16
**Branch:** main
**Status:** Architecture redesign complete. New spec written. F03/F04/F05 features defined. No code written yet.

## What exists

- Package scaffolded: `package.xml`, `setup.py`, `CLAUDE.md`, `.claude/`.
- `slam_manager_node.py` — monitors `/map`, publishes `/dome_nav/slam_status`, saves pose graph every 30s and on shutdown.
- `nav_manager_node.py` — subscribes `/intent`, sends `NavigateToPose` goals to Nav2.
- `utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()` for launch config merging.
- `config/slam_param_patch.yaml` — slam_toolbox overrides (range, TF tolerance, map persistence).
- `config/nav2_param_patch.yaml` — Nav2 overrides.
- `launch/robot.launch.py` — better_launch wrapper (slam_toolbox + Nav2 + manager nodes).
- `01-literate/` — literate docs for utils, slam_manager_node, nav_manager_node.
- `test/test_slam_manager.py`, `test/test_nav_manager.py` — unit tests (require rclpy, fail without ROS2).
- `02-doc/brainstorm.md` — full architecture brainstorm from 2026-06-16 session.

## Architecture Decision (2026-06-16)

**New approach: Static map + AMCL** replaces lifelong slam_toolbox as the navigation strategy.

Two modes:
- **Mode A (map build)**: `robot_map.launch.py` — slam_toolbox online_async, human teleoperation, save map on shutdown. Run once.
- **Mode B (navigate)**: `robot_nav.launch.py` — map_server + AMCL + Nav2. Normal operation. AMCL converges from lidar alone, no fixed start position, no fiducials needed.

Key reasons:
- AMCL (particle filter) provides global localization without known initial pose
- Static map = no TF jumps from loop closure, no Nav2 goal invalidation
- Much simpler, more robust, 20+ years battle-tested
- Outdoor extension: swap AMCL for GPS/RTK EKF, everything else unchanged

`spec.md` fully rewritten to reflect this.

## To switch to AMCL (implementation steps for F03)

1. Rename `robot.launch.py` → `robot_map.launch.py` (Mode A, unchanged)
2. New `robot_nav.launch.py`: replace `slam_toolbox online_async_launch.py` with
   `nav2_bringup localization_launch.py` (map_server + amcl)
3. Re-enable AMCL in `nav2_param_patch.yaml` (currently disabled — slam_toolbox owns
   `map→odom` TF; AMCL and slam_toolbox cannot coexist on that TF edge)
4. Remove `slam_manager_node` from nav launch (not needed in Mode B)
5. Note: AMCL particle filter converges from any start — no dock/initial pose needed

## dome_vision finding

`semantic_map_node.py` line 26: `ODOM_FRAME = "odom"` — objects stored in odom frame,
which resets each session. For cross-session object memory, must change to `"map"`.
This is NOT a dome_nav change — it's in dome_vision repo.

## F01/TF01 — Done

All tasks complete. slam_toolbox launches, `/map` publishes, TF valid, status publishes,
pose graph saves every 30s and on shutdown.

Key design: periodic save (30s) because SIGINT kills slam_toolbox before slam_manager
so shutdown save is unreliable. Timer-based save ensures persistence.

## F02/TF02 — Partially Done

- T01–T04 done (unit tests, routing, cancel, result callbacks)
- T05 not done (manual integration — needs live stack with Nav2 fully up)
- F02 needs revisiting: intent navigation now depends on AMCL mode (F03) not slam_toolbox

## Features Not Done

- **F03** — AMCL navigation mode: new `robot_nav.launch.py`, map_server + AMCL + Nav2
- **F04** — ROS-free unit tests: extract pure Python `slam_manager.py` / `nav_manager.py` from nodes
- **F05** — Sensor-only integration test: rosbag-based, no dome_vision/dome_control needed

## Likely Next Steps

1. F04 first: extract pure Python classes → enables tests without ROS
2. F03: write `robot_nav.launch.py` + AMCL config
3. F02 T05: retry live stack test once F03 is working
4. dome_vision: change `ODOM_FRAME = "odom"` → `"map"` in `semantic_map_node.py`
