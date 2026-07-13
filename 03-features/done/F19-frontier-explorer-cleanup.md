# F19 — Frontier Explorer Cleanup

**Priority**: Low
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Code-quality cleanup (review findings 6-8, 2026-07-13). No
behavior change.

## Scope

6. `pick_best_frontier` takes 11 parameters. All filter params come from
   `ExploreParams` at the sole production call site (`frontier_algorithm.py`).
   Collapse the signature to `(clusters, info, robot_xy, params, blacklist)` —
   read filter values off `params`.
7. `find_and_send_frontier` is ~73 lines. Extract the no-frontier branch
   (telemetry write + patience check + stop) into `handle_no_frontier(robot_xy)`.
8. `frontier_diag` recomputes cluster centroids inline; `explore_diagnostics.py`
   now owns `cluster_centroid`. `frontier_diag` does not actually need centroids
   (it filters by nearest cell + size), so no shared helper is required there —
   verify and leave as-is, or dedupe if a genuine centroid computation remains.

## Constraints

- No behavior change; existing tests pass (update call sites/signatures as needed).
- `frontier_explorer.py` stays pure Python, no ROS imports. `ExploreParams` lives
  in `explore_context.py` (also pure) so importing it there is allowed.

## How to Demo

`pytest test/ -m "not manual"` green. `pick_best_frontier` signature is <=5 params.
