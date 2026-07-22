# F30 — Path-Distance Frontier Ranking (Replace Euclidean)

**Priority**: High
**Done:** no
**Tasks File Created:** no
**Tests Written:** no
**Test Passing:** no

**Description**: Frontier selection ranks candidate cells by **straight-line
Euclidean distance** from the robot (`best_cell_in_cluster` in
`frontier_explorer.py`), scoring `|d - preferred_goal_distance|`. Euclidean
distance ignores walls: a frontier cell 1.0 m away on the far side of a wall
scores as a perfect preferred-distance match while the real drive is 8 m
around — or impossible. Consequences observed in real runs: through-wall goal
picks → `NO_VALID_PATH` / abort → blacklist churn → premature
`NO_TARGETS_BLOCKED`, the core of the stall pattern under investigation
(2026-07-18/20). This feature replaces the ranking metric with **true traversal
distance**: a single Dijkstra wavefront over free `/map` cells
(`path_distance_field`) computed once per goal-selection tick, giving path
distance from the robot to every reachable cell.

## Design sketch (from 2026-07-21 review)

- New pure function `path_distance_field(data, info, robot_xy) -> dict[int, float]`
  in `frontier_explorer.py`: Dijkstra from the robot cell over free cells
  (`data == 0`), 8-connected, diagonal steps cost `res·√2` so distances are
  metric. Cells absent from the dict are unreachable.
- `best_cell_in_cluster` takes the field: `d = dist_field.get(cell, inf)`;
  unreachable cells are **skipped entirely** — a frontier with no free path
  (closed door, far side of an unmapped wall) is never selected, so Nav2 never
  sees it and never aborts on it. Score formula unchanged: `|d - preferred|`.
- `best_frontier_candidates` computes the field once and threads it through;
  `map_data` added to the signature chain (the novelty path in `select_target`
  already has it).
- **Opt-in** `use_path_distance: bool = False` in `FrontierParams`, declared via
  `declare_frontier_params` — same pattern as F15 novelty. Default off ⇒
  existing behavior untouched.

## Expected wins

- Through-wall cells deprioritized by their real path length, not their
  fake Euclidean proximity.
- Unreachable frontiers drop out before dispatch — no `NO_VALID_PATH` abort,
  no blacklist churn from them. Removes a major stall input.
- `min_frontier_dist` / `max_frontier_dist` filters become path-metric —
  more honest meaning of "how far the robot must actually travel".

## Constraints / gotchas

- **CPU**: Dijkstra over all free cells, pure Python, per selection tick
  (~32k cells on a 9×9 m map @ 0.05 res). Runs only when picking a goal (not
  while a goal is active), so amortized cost is low; measure on the Pi anyway.
- **Robot cell not free** (transient SLAM state): snap the start to the nearest
  free cell within a small radius; if none, return an empty/None field and fall
  back to Euclidean for that tick — never a blocked tick from a bad start cell.
- **Semantics shift**: `preferred_goal_distance` (1.0 real / 2.0 sim) now means
  *path* meters; mild retune expected after A/B.
- Pure-Python module stays pure (no ROS/numpy imports) — F23 decoupling intact;
  the algorithm still reads only `/map` from `ExplorationContext`, no costmap.
- Novelty (F15) composes: path-distance shortlist feeds `pick_by_novelty`
  unchanged.

## Relation to other features

- Partially delivers F27's deferred "give the algorithm costmap awareness" idea,
  but via `/map` (already in ctx) — no protocol change.
- Shrinks F28's problem: fewer doomed goals selected ⇒ fewer reject/blacklist
  entries needing reason-tagging.

## How to Demo

**Setup**: sim stack
(`bl dome_nav sim_nav_full.launch.py --map_name f30test --world_name multi_room`),
two sessions: `--use_path_distance false` then `true`.

**Steps**:
1. `ros2 topic echo /explore/status`; watch node logs + telemetry.
2. Run exploration in the multi-room world until done in both sessions.
3. Compare: count of `NO_VALID_PATH`/aborted goals, blacklist size at session
   end, goals sent vs reached, time to completion.

**Expected output**: with `use_path_distance true`, no goals dispatched to
frontiers without a free path; fewer aborts and smaller final blacklist than the
Euclidean session; goals visibly on the robot's side of walls at comparable
preferred distance.
