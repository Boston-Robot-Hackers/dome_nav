# TF21 — Remove Dead None-Handling for F21

## T01 — Tighten node-side frontier-exhaustion signatures
\*\*Status\*\*: done
**Description**: `handle_no_frontier(robot_xy: XY)`, `dump_frontier_exhaustion(robot_xy: XY)`.
**Test**: existing node tests green.

## T02 — Tighten diagnostics formatters; use math.dist
\*\*Status\*\*: done
**Description**: `format_frontier_exhaustion(info: MapInfo, robot_xy: XY, ...)` —
drop dead `info is None` half. `exhaustion_cluster_line(robot_xy: XY, ...)` —
drop dead `if robot_xy else` fallbacks, use `math.dist`, nearest-cell via
comprehension + `min(default=inf)`.
**Test**: existing test_explore_diagnostics tests green.

## T03 — Update feature file and current.md
\*\*Status\*\*: done
