# TF19 — Frontier Explorer Cleanup for F19

## T01 — Collapse pick_best_frontier signature to use ExploreParams
**Status**: done
**Description**: `pick_best_frontier(clusters, info, robot_xy, params, blacklist=None,
start_xy=None)` — 11 params → 6. Filters read off `params` (ExploreParams).
`ExploreParams` imported under `TYPE_CHECKING` to avoid a circular import
(explore_context imports MapInfo from frontier_explorer). Also split the inner
loops into `cluster_outside_radius()` and `best_cell_in_cluster()` helpers.
Test call sites use a `filters()` helper defaulting `min_frontier_dist=0.0`
(geometry-friendly, unlike ExploreParams' operational 1.3 m default).
**Test**: 56 frontier tests green.

## T02 — Extract handle_no_frontier from find_and_send_frontier
**Status**: done
**Description**: Move the no-frontier branch (increment count, log, telemetry
write, patience check, dump + stop) into `handle_no_frontier(robot_xy)`.
find_and_send_frontier drops under 50 lines.
**Test**: node tests green (no behavior change).

## T03 — Verify finding 8 (frontier_diag centroid dup)
**Status**: done
**Description**: Confirmed `frontier_diag` computes no centroids (filters by
nearest-cell range + size via `cell_out_of_range`). The centroid duplication was
only across the two dump methods, already unified into `cluster_centroid` in
`explore_diagnostics.py` under F18. No change needed here.

## T04 — Update feature file and current.md
**Status**: done
