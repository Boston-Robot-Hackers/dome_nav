# F15 — Path Novelty Scoring for Frontier Selection

**Priority**: Medium
**Done:** no
**Tasks File Created:** no
**Tests Written:** no
**Test Passing:** no
**Description**: Score each frontier candidate by how much unknown space lies along
the straight-line path from the robot's current position to that candidate. The score
is the count of unknown cells (value -1 in the raw OccupancyGrid) crossed by a
Bresenham line from robot to candidate. Higher unknown-cell count = more new territory
revealed by travelling that path = better candidate. This score can be combined with
the distance score from F14 (or used standalone) to prefer paths that genuinely expand
the map rather than traversing already-explored corridors.

## Scope

- `dome_nav/frontier_explorer.py` — add `path_novelty_score(start_xy, end_xy, data,
  info) -> int` function: walks a Bresenham raster line between the two world
  coordinates, counts cells where `data[idx] == -1` (unknown). Pure Python, no ROS
  dependency. Add `score_frontier_candidates()` helper that returns a list of
  `(candidate_xy, novelty_score)` pairs for a set of pre-filtered candidates
  (after blacklist/distance/size filters have already been applied).
- `dome_nav/frontier_algorithm.py` — after `pick_best_frontier()` returns a candidate
  (or a short-list of candidates), optionally re-rank by novelty score. Controlled by
  `ExploreParams.use_novelty_scoring: bool = False` — opt-in so existing behaviour is
  unaffected by default.
- `dome_nav/explore_context.py` — add `use_novelty_scoring: bool = False` to
  `ExploreParams`.
- `dome_nav/pluggable_explore_manager_node.py` — declare ROS parameter
  `use_novelty_scoring` (default False). Wire into `ExploreParams`.
- Launch files — add `use_novelty_scoring` arg; default False for both sim and real
  until live-verified.
- Telemetry — add `novelty_score` field to `goal_sent` event so sessions with and
  without the feature can be compared.

## Algorithm detail

```
Bresenham line from robot_cell to candidate_cell:
  for each cell (r, c) on the line:
    if data[r * width + c] == -1:
      score += 1
return score
```

All arithmetic is on integer cell indices. Cell coordinates computed via inverse of
`cell_to_world()`. The line includes both endpoints.

When `use_novelty_scoring=True`, `frontier_algorithm.py` generates the top-N
candidates from `pick_best_frontier()` (where N is a small constant, e.g. 5) and
picks the one with the highest novelty score. Ties broken by preferred distance (F14)
or nearest if F14 is not in use.

## Constraints

- `path_novelty_score` must be pure Python with no ROS or numpy imports — it lives in
  `frontier_explorer.py` alongside the other pure functions.
- Scoring runs only on the short-list (≤5 candidates), not all frontier cells, to keep
  per-tick cost low. At 2 Hz and typical path lengths of 1–3 m (10–30 cells), total
  cost per tick is negligible.
- The feature is opt-in (`use_novelty_scoring=False` default) — existing exploration
  behaviour is completely unaffected unless the parameter is set.
- Novelty score is over the straight-line path, not the actual Nav2 planned path.
  This is intentional: the planned path is unavailable at goal-selection time, and the
  straight-line approximation is cheap and directionally correct.

## How to Demo

**Setup**: sim stack running (`bl dome_nav sim_nav_full.launch.py --map_name f15test
--world_name multi_room --use_novelty_scoring true`).

**Steps**:
1. `ros2 topic echo /explore/status`
2. `tail -f ~/.dome/telemetry/exp-*.json` — observe `novelty_score` field in
   `goal_sent` events.
3. Compare a session with `use_novelty_scoring false` vs `true` — later goals in the
   `true` session should have lower novelty scores (map filling in) while earlier ones
   are high.

**Expected output**: `goal_sent` events include `novelty_score > 0`; robot visibly
prefers paths into unmapped areas rather than re-traversing corridors it has already
scanned.
