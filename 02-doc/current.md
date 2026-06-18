# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-18
**Branch:** main
**Status:** F03 (AMCL/Mode B) and F07 (lifecycle + map persistence) both complete and
verified on live robot. Shutdown tracebacks fixed. Save-on-first-map-receipt added.

## What exists

- `dome_nav/nav_manager.py` — pure Python `NavManager`: JSON parse, nearest-target,
  localization score, status strings (KEPT — real algorithms, 21 pure tests)
- `dome_nav/slam_manager_node.py` — **LifecycleNode**: watches `/map`, saves pose graph
  on first map receipt + every 30s. Self-manages lifecycle (trigger_configure/activate in
  main(); better_launch lifecycle disabled via lifecycle_waittime=None).
- `dome_nav/nav_manager_node.py` — ROS2 node: `/intent` → NavigateToPose, status,
  `/amcl_pose` → localization status/score.
- `dome_nav/utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()`,
  `write_config()` (content-addressed launch cache)
- `config/` — slam_param_patch (map_start_pose: [0,0,0]), nav2_param_patch, nav2_amcl_patch
- `launch/robot_map.launch.py` (Mode A) — accepts `map_name` arg (default: basement1)
- `launch/robot_nav.launch.py` (Mode B) — AMCL + Nav2, verified working
- Tests: `test_nav_manager_pure.py` (21), `test_utils_pure.py` (5),
  `test_nav_manager.py` (18, ROS), `test_slam_manager.py` (11, ROS lifecycle),
  `test_map_validation.py` (4, manual/live only)

## Test status

**55 passed, 4 deselected** (manual) via
`python3 -m pytest src/dome_nav/test/ -m "not manual"`. The 4 manual tests need a live
stack. Build: `colcon build --packages-select dome_nav --symlink-install`.

## This session's work

- TF07 T04: verified map saved on Ctrl-C (Mode A). Fixed several issues along the way:
  - better_launch lifecycle conflict → lifecycle_waittime=None + self-managed transitions
  - save-on-first-map-receipt added (can't rely on shutdown save — race with slam_toolbox)
  - map_start_pose: [0,0,0] added to slam_param_patch (fixes LocalizationSlamToolbox warning)
  - map_name launch arg added to robot_map.launch.py
  - shutdown tracebacks suppressed in slam_manager and nav_manager
- TF03 T05: Mode B smoke test passed on live robot — AMCL pose at dock (x=-2.768,
  y=0.145), map→odom TF publishing, /amcl_pose live.

## Open issues (05-issues/)

- I02–I05: nav_manager crashes/silent-drops (non-list/non-dict JSON, missing xyz_world,
  silent intent drop) → TF02 T06–T09
- I06: leading-underscore MUST violations (3 source + 3 test files) → TF02 T10
- I07: localization score not clamped to 1.0 → TF06 T05
- I08: test files missing header → TF06 T06
- I09: `should_save()` was a 1-line method — now moot (folded into lifecycle node); verify
  before closing
- I10: `navigate_status()` defined but bypassed by node (dead + DRY) → not yet tasked

## Likely next steps

1. TF06 T05 + T06 — clamp score (5 lines) + add file headers (quick)
2. TF02 T06–T09 — nav_manager crash fixes (boundary validation)
3. TF02 T10 — underscore rename sweep
4. I10 — wire node to call `NavManager.navigate_status()` instead of inline strings
5. TF06 T04 / TF02 T05 — manual live-stack tests

## AMCL notes (unchanged)

- Convergence: `covariance[0]` (x) and `covariance[7]` (y) both < 0.05 m² = converged
- Foxglove: plot `/amcl_pose.pose.covariance[0]` and `[7]`
- `set_initial_pose: true` at basement1 dock pose (x=-2.768, y=0.145, yaw=1.743)
- Particle cloud topic: `nav2_msgs/msg/ParticleCloud` — Foxglove 3D panel does not render
  natively (changed from geometry_msgs/PoseArray in Jazzy)
