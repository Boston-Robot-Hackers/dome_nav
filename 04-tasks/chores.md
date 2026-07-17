# Chores

Running list of simple bug fixes and refactors (no spec/behavior change) that
don't warrant a feature/task pair. One line each; flip `- [ ]` to `- [x]` when
applied. A bug fix still gets a test.

## Todo

- [ ] If FOLLOW/TF_ERROR persists on real robot after transform_tolerance bump, lower `controller_frequency` 20→10 Hz in `nav2_params_explore_real.yaml` to relieve Pi MPPI CPU load (keeps TF fresher).
- [ ] Naming parallelism (F22 T01/R2): settle Explore vs Exploration across the seam — `ExploreParams`/`explore_tick`/`explore_algorithm`/`ExplorerManagerNode` (Explore) vs `ExplorationContext`/`ExplorationAlgorithm` (Exploration). Mechanical rename, no behavior change; update tests/imports. (Redundant `Pluggable` prefix already dropped from the node name.)

## Done

- [x] `explorer_manager_node.py` DRY/YAGNI pass (644→584 lines): extracted `dist()`/`rounded()` helpers, `call_hook()` (single optional-hook dispatch replacing 4 getattr wrappers), `abandon_active_goal()` (stuck/timeout cancel+blacklist+clear tail), `write_goal_result()` (3 duplicated telemetry writes). Dropped dead guards the state machine already guarantees (`last_progress_time`/`goal_start_time` None checks in the watchdogs), double algorithm-name validation (`resolve_algorithm` removed — `__init__` warns + falls back once at the param boundary), dead `start_xy` pre-capture in `on_intent` (tick owns it), unused `"succeeded"` in `GOAL_STATUS_NAMES`, duplicate `session_end` in `main()` (now only when state == exploring). Deleted `test_timeout_no_start_time_does_nothing` (encoded the dropped guard) and the 3 `resolve_algorithm` tests. Kept: malformed-JSON guard (true boundary), `paused_on_failure`/`exploration_resume` (resume is sent after stuck), costmap-None permissive default. Full suite 220 passed, 4 deselected.
- [x] `test_explorer_manager_node.py`: `test_unknown_explore_algorithm_param_falls_back_to_frontier` never injected an unknown `explore_algorithm` value (constructed the node with the default, so the warning assertion failed) — committed failing in the F22 close-out. Fixed by patching `Node.declare_parameter` to declare the param as `not_a_real_algorithm` during construction; full suite 211 passed, 4 deselected.
- [x] `frontier_explorer.py`: `find_frontier_clusters` clamped `max(1, buffer_cells)`, silently ignoring `buffer_cells=0`. Now 0 means boundary cells (touching unknown); added regression test.
- [x] `nav2_params_explore_real.yaml`: raise MPPI `FollowPath.transform_tolerance` 0.1→0.3 — Pi TF stale >0.1s under MPPI load caused FOLLOW/TF_ERROR aborting goals in 0.4s (real-robot run 2026-07-13).
