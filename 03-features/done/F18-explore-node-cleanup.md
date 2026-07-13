# F18 — Exploration Node Cleanup

**Priority**: Medium
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Code-quality cleanup of `pluggable_explore_manager_node.py` and
`nav_manager_node.py` found in the 2026-07-13 review. No behavior change.

## Scope

1. Extract the ~160 lines of diagnostic-dump formatting
   (`dump_frontier_exhaustion`, `dump_failure_diagnostics`, `costmap_radius_costs`,
   `costmap_cell_cost`) from `pluggable_explore_manager_node.py` into a new
   `dome_nav/explore_diagnostics.py`. Node drops from 677 to ~460 lines; the
   formatting becomes testable without ROS.
2. Remove the dead `centroid` parameter: it is always equal to `goal_xy` at the
   sole call site (`send_nav_goal(goal_xy, centroid=goal_xy)`). Collapse to `xy`
   throughout (`send_nav_goal`, `on_goal_accepted`, `on_goal_result`,
   `current_goal_centroid` → drop).
3. Replace `print(..., flush=True)` diagnostic output with the node logger.
4. Remove the dead `on_nav_feedback` no-op callback in `nav_manager_node.py` and
   its `feedback_callback=` registration.
5. Collapse the redundant `elif v == 255` / `elif v < 0` branches in
   `costmap_radius_costs` (identical output).

## Constraints

- No behavior change; existing tests must still pass.
- Diagnostics output text may change format slightly (logger vs print) but must
  still contain the same information.

## How to Demo

Run the full test suite (`pytest test/ -m "not manual"`) — all green. Node file
under 500 lines. `grep -n "print(" dome_nav/pluggable_explore_manager_node.py`
returns nothing.
