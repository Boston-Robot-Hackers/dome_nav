# Chores

Running list of simple bug fixes and refactors (no spec/behavior change) that
don't warrant a feature/task pair. One line each; flip `- [ ]` to `- [x]` when
applied. A bug fix still gets a test.

## Todo

- [ ] If FOLLOW/TF_ERROR persists on real robot after transform_tolerance bump, lower `controller_frequency` 20→10 Hz in `nav2_explore_real.yaml` to relieve Pi MPPI CPU load (keeps TF fresher).

## Done

- [x] `frontier_explorer.py`: `find_frontier_clusters` clamped `max(1, buffer_cells)`, silently ignoring `buffer_cells=0`. Now 0 means boundary cells (touching unknown); added regression test.
- [x] `nav2_explore_real.yaml`: raise MPPI `FollowPath.transform_tolerance` 0.1→0.3 — Pi TF stale >0.1s under MPPI load caused FOLLOW/TF_ERROR aborting goals in 0.4s (real-robot run 2026-07-13).
