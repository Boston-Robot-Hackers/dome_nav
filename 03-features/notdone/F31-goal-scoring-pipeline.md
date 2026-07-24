# F31 — Goal Scoring Pipeline (Filters + Weighted Scorers) + Obstacle Clearance

**Priority**: High
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no

**Description**: Goal selection has accreted three heuristics, each bolted on a
different way: distance-to-preferred lives inline in `best_cell_in_cluster`
(`frontier_explorer.py`), novelty (F15) is a two-stage short-list-then-re-rank
hack in `FrontierAlgorithm.select_target`, and the filters (size, blacklist,
min/max dist, max_radius, F27 lethal) are scattered `if params.x` guards. Adding
a fourth — **obstacle clearance**, needed because frontier cells buffer only
against *unknown* (`find_frontier_clusters`), never against *occupied*, so goals
hug walls and feed the F29 wedge — the same bolt-on way means more sprawl. This
feature refactors selection into one **filters + weighted scorers** pipeline (the
shape Nav2's own MPPI `CriticManager` uses), then adds obstacle clearance as its
first new tenant to prove the pipeline composes.

## Problem being fixed (observed 2026-07-22)

Frontiers form exactly where free meets unknown, and walls cluster there too.
`buffer_cells=2` keeps a goal ~0.1 m (at 0.05 res) clear of the **unknown seam**
but gives **zero clearance from walls**. F27 only rejects goals *on* lethal
cells; near-lethal passes. Result: picked targets sit hard against obstacles →
Nav2 approaches a wall-adjacent goal → collision_monitor `FootprintApproach`
gates → the wedge under F29 investigation. The desired pick is: **far along a
frontier, but as far from obstacles as the local geometry allows.**

## Architecture: filters + weighted scorers

Two roles, both operating on a per-cell context struct precomputed once per tick:

```python
@dataclass
class CellCtx:            # everything a heuristic might read, computed once
    xy: tuple[float, float]
    cell_idx: int
    d_robot: float        # or path distance if F30 on
    clearance: float      # cells to nearest occupied, from the clearance field
    data: Sequence[int]
    info: MapInfo
    robot_xy: tuple[float, float]
    start_xy: tuple[float, float] | None
    params: FrontierTuning

Filter = Callable[[CellCtx], bool]      # True = keep; hard reject, no weight
Scorer = Callable[[CellCtx], float]     # cost, lower = better, normalized [0,1]
```

Selection collapses to one loop:

```python
def pick(cells, filters, scorers):
    best, best_cost = None, inf
    for c in cells:
        if not all(f(c) for f in filters):
            continue
        cost = sum(w * s(c) for w, s in scorers)   # enabled scorers only
        if cost < best_cost:
            best_cost, best = cost, c.xy
    return best
```

A **registry** built from `FrontierParams` lists every heuristic (name, role,
weight, enabled) in one place — the single spot to read what drives selection.

## Heuristic inventory (where each lands)

| Heuristic | Role | Notes |
|---|---|---|
| min_frontier_size | cluster filter | stays cluster-level pre-pass |
| max_explore_radius | cluster filter | centroid vs start_xy |
| blacklist | cell filter | |
| min/max_frontier_dist | cell filter | |
| lethal (F27) | cell filter | |
| **clearance floor** (new) | cell filter | `clearance ≥ R_inscribed + margin` |
| distance-to-preferred | scorer | `abs((d - inset) - preferred)` |
| novelty (F15) | scorer | **two-stage hack removed** — just a weighted scorer |
| **clearance bonus** (new) | scorer | `-clearance`, rewards open cells |
| path-distance (F30) | filter + `d` source | unreachable → cell filter drop; feeds `d_robot` |

## Obstacle clearance — the new primitive

**Clearance field**: for each free cell, distance (in cells) to nearest occupied
cell. One BFS per map update — mark all occupied (`data ≥ LETHAL_THRESHOLD`) as
sources, flood outward. O(N) one pass; per-candidate lookup is O(1). New pure
function `clearance_field(data, info) -> Sequence[float]` in
`frontier_explorer.py`, mirroring F30's `path_distance_field` idiom.

- **Floor filter**: reject cells with `clearance < R_inscribed + margin`.
  Reachability only — keeps goals off walls without starving corridors.
- **Bonus scorer**: `-clearance` (negated so lower cost = more open), normalized.
  Breaks ties toward open space among survivors.

## Two things to get right

1. **Normalization is mandatory.** Distance is meters, novelty is cell-counts,
   clearance is cells — different scales, so raw weights are meaningless. Each
   scorer normalizes to [0,1] per cycle (min-max over surviving candidates)
   *before* weighting. Weights become comparable, tunable numbers.
2. **Keep the cluster/cell two-phase split.** Size + max_radius are cluster-level
   (centroid); run them as a cluster pre-filter, then flatten survivors to cells
   for cell filters + scorers. Don't force cluster predicates into the cell list.

## Convention

All scorers return **cost, lower = better, normalized [0,1]**; minimize the
weighted sum. This matches the existing `goal_score` min-convention, so
`best_cell_in_cluster`'s core survives, just generalized. Rewards (clearance,
novelty) negate before normalizing.

## Why this kills the special cases

- **Novelty stops being special.** No short-list-then-re-rank; the
  `select_target` branch disappears. `use_novelty_scoring` just sets its weight.
- **New heuristic = register one object.** Clearance adds one filter + one scorer
  to the registry. No new plumbing, no new branch. Same for any future heuristic.
- **One place to read all heuristics** — the registry, built from
  `FrontierParams`. Config maps name → weight/enabled.

## Constraints / gotchas

- **Behavior parity is the regression anchor.** With clearance weight 0 and F15
  novelty migrated in, the pipeline must reproduce the old two-stage selection
  cell-for-cell on fixed fixtures — that test proves the refactor is safe before
  any tuning.
- **Clearance floor must stay low** (inscribed radius + small margin). A high
  floor starves corridors (max clearance there is half the corridor width) → no
  valid goal → stall. The *bonus* does the "prefer open" work, not the floor.
- **Pure-Python module stays pure** — no ROS/numpy imports; F23 decoupling
  intact. Still reads only `/map` from `ExplorationContext`, no costmap subscribe.
- **CPU**: clearance BFS is O(N) over the grid (~32k cells, 9×9 m @ 0.05),
  per map update, pure Python. Cheaper than F30's Dijkstra; measure on the Pi.
- **Weights are new tunables** in `FrontierParams`, declared via
  `declare_frontier_params` — same opt-in pattern as F15/F30. Defaults chosen so
  clearance weight > 0 (fixes the observed wall-hug) but novelty/distance keep
  their current effective behavior.

## Relation to other features

- **F15** (novelty) and **F30** (path-distance) are the other two heuristics; this
  pipeline is their common home. F30's `path_distance_field` and this feature's
  `clearance_field` are both per-cycle precomputed fields feeding `CellCtx`.
  Ordering: land F31 as the frame, then F15/F30 migrate in as scorers/filters
  rather than more bolt-ons. If F30 ships first, F31 absorbs it.
- **F27** lethal guard becomes a registered cell filter — no behavior change,
  just relocation.
- **F29**: fewer wall-hugging goals dispatched ⇒ fewer collision_monitor wedges
  to escape. Attacks the wedge at its input, upstream of the BackUp recovery.
- **F28**: fewer doomed goals ⇒ fewer reject/blacklist entries to reason-tag.

## How to Demo

**Setup**: sim stack
(`bl dome_nav sim_nav_full.launch.py --map_name f31test --world_name multi_room`),
two sessions: clearance weight 0 (baseline) then the tuned default.

**Steps**:
1. `ros2 topic echo /explore/status`; watch node logs + telemetry
   (`tail -f ~/.dome/telemetry/e*.json`).
2. Run exploration in the multi-room world to completion in both sessions.
3. Compare picked-goal clearance (distance to nearest wall at dispatch),
   collision_monitor `FootprintApproach` gate events, aborts, and final
   blacklist size.

**Expected output**: with clearance enabled, dispatched goals sit visibly off
walls (higher measured clearance), fewer `FootprintApproach` gate events / wedges,
without the corridor cases stalling for lack of a valid goal. Baseline-weight-0
session reproduces current wall-hugging behavior — confirming the pipeline is
behavior-preserving and clearance is the only new lever.
