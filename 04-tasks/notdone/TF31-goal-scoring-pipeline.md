# TF31 — Goal Scoring Pipeline + Obstacle Clearance for F31

Refactor goal selection into **filters + weighted scorers** (one `CellCtx`,
one registry), then add obstacle clearance as the first new tenant. Migration
order is deliberate: land the frame behavior-preserving (T01–T03), then add the
new lever (T04–T05). All new code stays in the pure `frontier_explorer.py` /
`frontier_params.py` modules — no ROS/numpy imports, F23 decoupling intact.

Reconciliation with current code: existing selection is
`best_frontier_candidates` → `best_cell_in_cluster` (distance scorer + inline
filters) with a two-stage novelty branch in `FrontierAlgorithm.select_target`.
The pipeline subsumes both. Cluster-level predicates (`min_frontier_size`,
`max_explore_radius`) stay a pre-pass; cell-level filters + scorers run per cell.

## T01 — Pipeline primitives (CellCtx, Filter/Scorer, pick loop, registry)
**Status**: done
**Description**: In `frontier_explorer.py` add `CellCtx` dataclass (xy, cell_idx,
d_robot, clearance, data, info, robot_xy, start_xy, params), the `Filter =
Callable[[CellCtx], bool]` / `Scorer = Callable[[CellCtx], float]` aliases, and
`select_cell(cells, filters, scorers) -> tuple[float,float]|None` — keep cells
passing `all(filters)`, minimize `sum(w*s(c) for w,s in scorers)`. Add a
`build_registry(tuning) -> (cluster_filters, cell_filters, scorers)` factory that
assembles the enabled heuristics from `FrontierTuning`. No behavior wired yet;
primitives + registry only.
**Test**: `select_cell` picks min weighted cost; a failing filter excludes a
cell; empty/all-filtered ⇒ None; weighted sum honors weights; `build_registry`
lists expected heuristics for a given tuning.

## T02 — Migrate existing filters + distance scorer (behavior-preserving)
**Status**: done
**Description**: Reexpress today's logic as registry entries: cell filters
`blacklist`, `min/max_frontier_dist` (F27 lethal filter too if present); scorer
`distance_to_preferred` = `abs((d - goal_inset_m) - preferred)` normalized [0,1]
per cycle. Cluster pre-pass keeps `min_frontier_size` + `max_explore_radius`.
Rewire `best_frontier_candidates` / `pick_best_frontier` onto `select_cell`.
**Test — PARITY (the anchor)**: on fixed cluster/map fixtures, pipeline output
equals the pre-refactor `pick_best_frontier` cell-for-cell across the existing
frontier_explorer test cases (novelty off, clearance weight 0).

## T03 — Migrate F15 novelty as a scorer (remove the two-stage hack)
**Status**: done
**Description**: Turn `path_novelty_score` into a `novelty` scorer (cost =
`-unknown_count`, normalized). Delete the short-list-then-re-rank branch in
`FrontierAlgorithm.select_target`; `use_novelty_scoring` now sets the novelty
scorer's weight (0 when off). `novelty_top_n` retires (or stays as a no-op with a
deprecation note). `latest_novelty` telemetry still populated from the winning
cell's raw unknown count.
**Test**: `use_novelty_scoring=False` ⇒ identical to T02 parity result;
`=True` ⇒ prefers higher-unknown-path cell; `telemetry_extra` carries
`novelty_score` only when enabled (matches TF15 T03 assertions).

## T04 — clearance_field + floor filter + bonus scorer
**Status**: done — full suite green (266 pass via `/usr/bin/python3 -m pytest`,
excl. 4 live-stack `test_map_validation` tests)
**Description**: Add pure `clearance_field(data, info) -> Sequence[float]`: BFS
from all occupied cells (`data >= LETHAL_THRESHOLD`), 8-connected, diagonal step
`√2`, value = cells-to-nearest-occupied (free/unknown cells filled; occupied = 0).
Computed once per tick, threaded into `CellCtx.clearance`. Add cell filter
`clearance_floor` (reject `clearance*res < R_inscribed + margin`) and scorer
`clearance_bonus` (cost = `-clearance`, normalized). Register both.
**Test**: clearance is 0 at occupied, grows with distance, metric diagonals;
floor rejects sub-margin cells, keeps clear ones; bonus ranks the more-open of
two equal-distance cells; a corridor (max clearance < margin nowhere-valid guard)
does not wipe out all candidates when margin is set at inscribed radius.

## T05 — Wire params + normalization into FrontierParams/Tuning/declare
**Status**: done — R_inscribed derived from a new `robot_radius` frontier param
(default 0.17); `clearance_margin_m`, `w_distance`/`w_novelty`/`w_clearance` added
and declared; default `w_clearance` 1.0 (clearance on) — needs T07 sim tuning.
Per-cycle min-max normalization confirmed in `select_cell`/`_minmax_normalize`
(inf-safe). merge round-trip test added; full suite green.
**Description**: Add to `FrontierParams`/`FrontierTuning`/`merge_tuning`/
`declare_frontier_params`: `clearance_margin_m: float`, `w_distance`,
`w_novelty`, `w_clearance` (weights; novelty weight gated by
`use_novelty_scoring`), `robot_inscribed_m` (or derive from robot_radius).
Confirm per-cycle [0,1] min-max normalization lives in the scorers/`select_cell`
path so weights are scale-comparable. Defaults: clearance weight > 0 (fixes
wall-hug), distance/novelty tuned to keep current effective behavior.
**Test**: defaults round-trip through declare/merge; normalization maps a
candidate set to [0,1]; changing `w_clearance` shifts the winner toward open
cells on a fixture; `merge_tuning` blacklist_radius > goal_inset_m invariant
still enforced.

## T06 — Feature file + literate regen
**Status**: done
**Description**: Set F31 flags; updated `01-literate/06-frontier_explorer.md`
(v1.2) and `01-literate/08-frontier_algorithm.md` (v1.1) for the F31 pipeline,
clearance field, and new params. Noted F15 two-stage retirement / `novelty_top_n`
deprecation in both the feature file and the algorithm literate doc.

## T07 — Sim verification
**Status**: not done — sim, manual
**Description**: `sim_nav_full.launch.py` multi_room, two sessions
(`w_clearance` 0 baseline vs tuned default). Confirm: goals dispatched off walls
(higher measured clearance), fewer `FootprintApproach` gate events, corridors
still yield goals. Baseline reproduces wall-hug ⇒ pipeline behavior-preserving.

## T08 — Live verification
**Status**: not done — hardware, manual
**Description**: Real robot, start near a wall. Confirm clearance-weighted goals
reduce the start-wedged stall (F29 input reduction) vs a `w_clearance 0` run.
Pairs with the F29 gate-probe outcome.
