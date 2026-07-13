# F21 — Remove Dead None-Handling in Frontier-Exhaustion Diagnostics

**Priority**: Low
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: The frontier-exhaustion dump path receives a non-None `robot_xy`
and non-None `MapInfo` (guaranteed by guards in `find_and_send_frontier`), yet the
formatters carry `| None` annotations and dead `if robot_xy else ...` fallbacks —
carried over verbatim when the diagnostics were extracted (F18). Tighten the whole
path; no behavior change.

## Scope

- `pluggable_explore_manager_node.py` — `handle_no_frontier(robot_xy: XY)` and
  `dump_frontier_exhaustion(robot_xy: XY)` drop `| None` (sole caller guards None
  upstream and returns).
- `explore_diagnostics.py` — `format_frontier_exhaustion(info: MapInfo,
  robot_xy: XY, ...)` drop `| None`; remove the dead `info is None` half of its
  guard (keep `not clusters`). `exhaustion_cluster_line(robot_xy: XY, ...)` drop
  `| None` and the three dead `if robot_xy else ...` fallbacks; use `math.dist`.

## Constraints

- No behavior change. `format_failure_diagnostics` and `costmap_cell_cost` keep
  their genuine `| None` (that path is reached with None robot_xy/costmap).

## How to Demo

`pytest test/ -m "not manual"` green (existing diagnostics tests pass non-None args).
