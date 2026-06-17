# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-17
**Branch:** main
**Status:** F03 (AMCL mode) live-tested and working. F04 (pure Python tests) done. F06 feature file created.

## What exists

- `dome_nav/slam_manager.py` — pure Python `SlamManager`: map readiness state, save gating, dir setup
- `dome_nav/nav_manager.py` — pure Python `NavManager`: JSON parsing, target selection, status strings
- `dome_nav/slam_manager_node.py` — thin ROS2 wrapper delegating to `SlamManager`
- `dome_nav/nav_manager_node.py` — thin ROS2 wrapper delegating to `NavManager`
- `dome_nav/utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()`
- `config/slam_param_patch.yaml` — slam_toolbox overrides
- `config/nav2_param_patch.yaml` — Nav2 overrides + stubs for `FootprintApproach.max_points`, `docking_server.dock_database`
- `config/nav2_amcl_patch.yaml` — AMCL + map_server params for Mode B; `set_initial_pose: true` at (0,0)
- `launch/robot_map.launch.py` — Mode A: slam_toolbox + Nav2 + both manager nodes
- `launch/robot_nav.launch.py` — Mode B: map_server + AMCL + Nav2 + nav_manager node
- `01-literate/` — literate docs for utils, slam_manager_node, nav_manager_node, slam_manager, nav_manager
- `test/test_slam_manager_pure.py` (8 tests), `test/test_nav_manager_pure.py` (15 tests) — no rclpy, 0.04s
- `test/test_slam_manager.py`, `test/test_nav_manager.py` — ROS node tests (require rclpy)

## Architecture

Two modes:
- **Mode A (map build)**: `bl dome_nav robot_map.launch.py` — slam_toolbox online_async + Nav2
- **Mode B (navigate)**: `bl dome_nav robot_nav.launch.py` — map_server + AMCL + Nav2

AMCL notes:
- `set_initial_pose: true` at (0,0) — only reliable if robot starts at map origin
- Convergence check: `covariance[0]` (x) and `covariance[7]` (y) — both < 0.05 m² = converged
- Foxglove: plot `/amcl_pose.pose.covariance[0]` and `/amcl_pose.pose.covariance[7]`

## Test Commands

```bash
# Pure tests (no ROS needed)
cd ros2_ws && source install/setup.bash
python3 -m pytest src/dome_nav/test/test_slam_manager_pure.py src/dome_nav/test/test_nav_manager_pure.py -v

# Launch Mode B
bl dome_nav robot_nav.launch.py
```

## dome_vision finding

`semantic_map_node.py` line 26: `ODOM_FRAME = "odom"` — objects stored in odom frame,
resets each session. Change to `"map"` for cross-session object memory. NOT a dome_nav change.

## F01/TF01 — Done

All tasks complete. slam_toolbox launches, `/map` publishes, TF valid, pose graph saves
every 30s and on shutdown.

## F02/TF02 — Partially Done

- T01–T04 done (unit tests, routing, cancel, result callbacks)
- T05 not done (manual integration — needs live Nav2 stack with confirmed targets)

## F03/TF03 — Done

- T01–T04 done (launch files, AMCL config, live test)
- T05 manual test: Mode B stack up, map loads, AMCL converges (needs correct initial pose)
- Key fix: removed `collision_monitor` section from patch to avoid `observation_sources: []` crash
- Key fix: `set_initial_pose: true` so AMCL starts without waiting for `/initialpose` topic

## F04/TF04 — Done

- Pure Python `SlamManager` and `NavManager` extracted from nodes
- 23 tests, all pass, no rclpy dependency, run in 0.04s
- Nodes refactored as thin delegation wrappers

## F06 — Planned

`03-features/notdone/F06-localization-status.md` created.
Add `NavManager.check_localization(covariance) -> str` + node subscribes `/amcl_pose`,
publishes `/dome_nav/localization_status` with `"converged"` or `"localizing"`.
Threshold: `covariance[0] < 0.1` and `covariance[7] < 0.1`.

## Likely Next Steps

1. F06: implement `check_localization` + amcl_pose subscription in nav_manager_node
2. F02 T05: retry live stack test with Nav2 fully up
3. Determine true dock/start coordinates in basement1 map for reliable AMCL init
4. dome_vision: change `ODOM_FRAME = "odom"` → `"map"` in `semantic_map_node.py`
