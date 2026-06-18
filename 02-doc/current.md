# dome_nav — Current Session Handoff

## Snapshot

**Date:** 2026-06-17
**Branch:** main
**Status:** Code review of whole package done (style_guide.md). F07 shipped
(lifecycle node + temp-file leak fix). 10 issues filed (I01–I11, no I-gap); I01 + I11
resolved. F08 typed-messages proposal written (not started).

## What exists

- `dome_nav/nav_manager.py` — pure Python `NavManager`: JSON parse, nearest-target,
  localization score, status strings (KEPT — real algorithms, 21 pure tests)
- `dome_nav/slam_manager_node.py` — **LifecycleNode**: watches `/map`, persists pose
  graph; synchronous save on shutdown. `SlamManager` pure class was folded in and deleted.
- `dome_nav/nav_manager_node.py` — ROS2 node: `/intent` → NavigateToPose, status,
  `/amcl_pose` → localization status/score. Property-proxies removed.
- `dome_nav/utils.py` — `dome_home()`, `yaml_override()`, `yaml_patch_dict()`,
  `write_config()` (content-addressed launch cache, replaces leaking temp files)
- `config/` — slam_param_patch, nav2_param_patch, nav2_amcl_patch (initial pose set to
  basement1 dock: x=-2.768, y=0.145, yaw=1.743)
- `launch/robot_map.launch.py` (Mode A), `launch/robot_nav.launch.py` (Mode B)
- Tests: `test_nav_manager_pure.py` (21), `test_utils_pure.py` (5),
  `test_nav_manager.py` (18, ROS), `test_slam_manager.py` (11, ROS lifecycle),
  `test_map_validation.py` (4, manual/live only)

## Test status

**55 passed, 4 deselected** (manual) via
`python3 -m pytest src/dome_nav/test/ -m "not manual"`. The 4 manual tests need a live
stack. Build: `colcon build --packages-select dome_nav --symlink-install`.

## This session's work

- Renamed `.claude/codereview.md` → `.claude/style_guide.md`; updated `/start`,
  `CLAUDE.md`, and the j3 template repo (committed + pushed to j3).
- Full code review against style_guide → issues I01–I11.
- F07 T01 (I11 temp-file leak) + T02 (I01 lifecycle) done. T03 (nav lifecycle) deferred.
  T04 (manual shutdown verify) pending live stack.
- F04 marked partially reversed (SlamManager extraction undone, with rationale).

## Open issues (05-issues/)

- I01 RESOLVED (F07 T02), I11 RESOLVED (F07 T01)
- I02–I05: nav_manager crashes/silent-drops (non-list/non-dict JSON, missing xyz_world,
  silent intent drop) → tasks TF02 T06–T09
- I06: leading-underscore MUST violations (3 source + 3 test files) → TF02 T10
- I07: localization score not clamped to 1.0 → TF06 T05
- I08: test files missing header → TF06 T06
- I09: `should_save()` was a 1-line method — now moot (folded into lifecycle node); verify
  before closing
- I10: `navigate_status()` defined but bypassed by node (dead + DRY) → not yet tasked

## Likely next steps

1. Fix the nav_manager crash class (I02/I03/I04/I05 — boundary validation + logging)
2. I06 underscore rename sweep (touches source + tests)
3. I10: wire node to call `NavManager.navigate_status()` instead of inline strings
4. F02 T05 / F03 T05 / F06 T04 / TF07 T04 — manual live-stack tests
5. F08 typed messages — needs dome_control + dome_vision in lockstep

## AMCL notes (unchanged)

- Convergence: `covariance[0]` (x) and `covariance[7]` (y) both < 0.05 m² = converged
- Foxglove: plot `/amcl_pose.pose.covariance[0]` and `[7]`
- `set_initial_pose: true` at basement1 dock pose
