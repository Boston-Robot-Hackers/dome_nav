# Chores

Running list of simple bug fixes and refactors (no spec/behavior change) that
don't warrant a feature/task pair. One line each; flip `- [ ]` to `- [x]` when
applied. A bug fix still gets a test.

## Todo

- [ ] If FOLLOW/TF_ERROR persists on real robot after transform_tolerance bump, lower `controller_frequency` 20→10 Hz in `nav2_params_explore_real.yaml` to relieve Pi MPPI CPU load (keeps TF fresher).
- [ ] Naming parallelism (F22 T01/R2): settle Explore vs Exploration across the seam — `ExploreParams`/`explore_tick`/`explore_algorithm`/`ExplorerManagerNode` (Explore) vs `ExplorationContext`/`ExplorationAlgorithm` (Exploration). Mechanical rename, no behavior change; update tests/imports. (Redundant `Pluggable` prefix already dropped from the node name.)

## Done

- [x] `frontier_explorer.py`: `find_frontier_clusters` clamped `max(1, buffer_cells)`, silently ignoring `buffer_cells=0`. Now 0 means boundary cells (touching unknown); added regression test.
- [x] `nav2_params_explore_real.yaml`: raise MPPI `FollowPath.transform_tolerance` 0.1→0.3 — Pi TF stale >0.1s under MPPI load caused FOLLOW/TF_ERROR aborting goals in 0.4s (real-robot run 2026-07-13).
