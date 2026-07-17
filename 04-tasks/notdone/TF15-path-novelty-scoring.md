# TF15 — Path Novelty Scoring for F15

Post-F23 reconciliation: the spec's file/param names predate F23. Actual targets:
node is `explorer_manager_node.py`; frontier tuning lives in `FrontierParams`
(`frontier_params.py`), not `ExploreParams`; the node stays frontier-decoupled, so
novelty telemetry rides the opaque `telemetry_extra` hook, not the node's `goal_sent`.

## T01 — Pure novelty function in frontier_explorer.py
**Status**: done
**Description**: Add `path_novelty_score(start_xy, end_xy, data, info) -> int`:
world→cell via floor inverse of `cell_to_world`, Bresenham integer line incl. both
endpoints, count cells where `data[idx] == -1`; out-of-bounds cells skipped. Pure
Python, no ROS/numpy.
**Test**: horizontal/vertical/diagonal lines; counts only `-1`; endpoints included;
zero when path crosses no unknown; out-of-bounds cells ignored.

## T02 — Candidate short-list + novelty picker
**Status**: done
**Description**: Refactor `pick_best_frontier` to delegate to new
`best_frontier_candidates(..., top_n)` (one best cell per cluster, distance-ranked;
`top_n=1` reproduces old behaviour exactly). Add `pick_by_novelty(candidates,
robot_xy, data, info) -> (xy|None, score)` — max novelty, ties keep input (distance)
order.
**Test**: `best_frontier_candidates` returns ≤top_n distance-ranked; `pick_best_frontier`
unchanged for top_n=1; `pick_by_novelty` picks max-unknown path, tie-breaks by order,
returns (None,0) on empty.

## T03 — Wire opt-in into FrontierAlgorithm + params
**Status**: done
**Description**: `FrontierParams`/`FrontierTuning`/`merge_tuning`/`declare_frontier_params`
gain `use_novelty_scoring: bool = False`, `novelty_top_n: int = 5`. Algorithm
`select_target` branches: default path unchanged; novelty path short-lists
`novelty_top_n` and re-ranks. Stash `latest_novelty`; expose via `telemetry_extra`
(`novelty_score`) only when set. Default off ⇒ existing behaviour untouched.
**Test**: `use_novelty_scoring=False` ⇒ same goal as before; `=True` ⇒ prefers
higher-unknown path; `telemetry_extra` carries `novelty_score` only when enabled.

## T04 — Update F15 feature file + literate
**Status**: done (feature file) — literate regen pending before PR
**Description**: Correct stale filenames/param location in `F15-*.md`; set done flags.
Regenerate literate for changed source before PR. Launch args deferred: param is
algorithm-declared and overridable via `--use_novelty_scoring`; feature is opt-in and
not live-verified.

## T05 — Live verification
**Status**: not done — hardware/sim, deferred
**Description**: Run sim with `use_novelty_scoring true`; confirm `novelty_score` in
telemetry and visibly-better frontier choices vs a `false` session. Manual.
