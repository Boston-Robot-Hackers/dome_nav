# TF18 — Exploration Node Cleanup for F18

## T01 — Extract diagnostics to explore_diagnostics.py
**Status**: done
**Description**: Move `dump_frontier_exhaustion`, `dump_failure_diagnostics`,
`costmap_radius_costs`, `costmap_cell_cost` into `dome_nav/explore_diagnostics.py`
as pure functions taking the data they need (clusters, costmaps, params). Node
calls them, passing state.
**Test**: Existing suite green after rebuild.

## T02 — Remove dead centroid parameter
**Status**: done
**Description**: Collapse `centroid` (always == `goal_xy`) to `xy` in
`send_nav_goal`, `on_goal_accepted`, `on_goal_result`, `check_goal_timeout`,
`clear_active_goal`. Blacklist uses `xy`.
**Test**: Existing suite green; add/adjust node tests if any reference centroid.

## T03 — Replace print with logger; drop dead feedback callback; fix redundant branch
**Status**: done
**Description**: (3) `print(..., flush=True)` → `self.get_logger()`; (4) remove
`on_nav_feedback` no-op + `feedback_callback=` in `nav_manager_node.py`; (5)
collapse `elif v == 255` / `elif v < 0` in `costmap_radius_costs`.
**Test**: `grep print(` returns nothing in the node; suite green.

## T04 — Update feature file and current.md
**Status**: done
**Note**: Review findings 6-8 (long param list on `pick_best_frontier`, split
`find_and_send_frontier`, `cluster_centroid` DRY in `frontier_diag`) are NOT in
F18 scope — `cluster_centroid` now exists in `explore_diagnostics.py` and could
absorb `frontier_diag`'s copy in a follow-up.
