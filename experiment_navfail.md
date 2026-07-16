# Experiment log — fail-fast target reselection (stop retrying hopeless goals)

Separate from experiment.md (which covers the deadband bug, the start-in-inflation
deadlock, and the Pi CPU campaign). This log is about the EXPLORER's response to
navigation failure: notice failure early and pick a different target instead of
grinding on a goal that has no hope.

## Problem
The explorer keeps a goal active until it succeeds, aborts, or hits the 25s
`GOAL_TIMEOUT_S`. When the robot is wedged (collision_monitor gate / start-occupied
deadlock — see experiment.md bug 2), that means ~25s of sitting still per hopeless
goal before moving on. We want to abandon fast and reselect.

## Two distinct failure cases (different detection + response)

### Case A — no suitable targets
The frontier selector returns nothing sendable this tick.
- Detected in `find_and_send_frontier` -> `handle_no_frontier`.
- Today: increments `no_frontier_count`, declares `done` at `NO_FRONTIER_PATIENCE`
  (14). This CONFLATES two situations:
  - **raw clusters == 0** (`algorithm.latest_clusters` empty) -> genuinely fully
    explored -> `done` is correct.
  - **raw clusters > 0 but all filtered / blacklisted / outside costmap /
    unreachable** -> NOT done, we are BLOCKED, not finished. Declaring done here
    ends exploration prematurely.
- Proposed response for the blocked sub-case: before giving up, age-out the
  oldest blacklist entries (targets blacklisted early may now be reachable as the
  map/costmap grew) and/or relax a filter, with a hard cap so it can't loop
  forever. Only declare `done` when raw clusters == 0.
- [VERIFIED] The signal exists: `algorithm.latest_clusters` (frontier_algorithm.py:29)
  is the RAW clusters before size/blacklist/dist filtering, and handle_no_frontier
  already logs `raw_clusters=len(...)`. Only the DECISION conflates the two cases
  (patience declares done regardless of raw count). Fix is pure node logic.

### Case B — target exists but robot is not moving (fail-fast stuck detection)
Goal accepted, but no forward progress (the deadlock/gate case).
- Today only `GOAL_TIMEOUT_S = 25s` catches it. Too slow.
- Add a NO-PROGRESS monitor while `has_active_goal`. Data already on hand:
  `robot_xy_in_map()`, `current_goal_xy`, `goal_start_time`.
- Track:
  - `best_dist_to_goal` — smallest distance-to-goal seen this goal.
  - `last_progress_xy`, `last_progress_time`.
- Each tick while active:
  - `d = dist(robot, goal)`; `moved = dist(robot, last_progress_xy)`.
  - PROGRESS if `d < best_dist - PROGRESS_EPS` OR `moved > MOVE_EPS`:
    update best_dist / last_progress_xy / last_progress_time.
  - else if `now - last_progress_time > STUCK_T`: declare STUCK.
- On STUCK: `cancel_goal_async`, blacklist target WITH radius, clear active goal,
  reselect next tick. ~4x faster than the 25s timeout.
- Keep `GOAL_TIMEOUT_S` as the hard cap (covers slow-but-not-stuck edge cases).

Proposed constants (tune on hardware):
| Name          | Start value | Meaning |
|---------------|-------------|---------|
| STUCK_T       | 6-8 s       | no-progress window before abandon |
| MOVE_EPS      | 0.05 m      | min translation counted as progress |
| PROGRESS_EPS  | 0.10 m      | min distance-to-goal drop counted as progress |

Edge case to guard: final in-place rotation near the goal makes ~0 translation
but IS progress. distance-to-goal barely changes too. Mitigate: only run stuck
detection while `d` is above the goal tolerance (i.e. still en route, not doing
the final align), or treat "within goal tolerance" as success-imminent.

## Blacklist must be a REGION, not a point  [VERIFIED — already works]
`ExploreParams.blacklist_radius = 0.5` IS enforced on the live path:
next_goal -> pick_best_frontier -> best_cell_in_cluster (frontier_explorer.py:178)
excludes any frontier cell within `blacklist_radius` of ANY blacklisted point.
So abandoning a target already suppresses its neighborhood, not just the exact XY.
Case B stuck-abandon needs no algorithm change — just `blacklist.add(target)`.
Not yet supported: a per-cause radius (wider for controller/collision failures
than planner "no path"). `br` is a single param; threading a per-add radius is a
small extra if we want it.

## Bonus: classify by Nav2 error code (already captured)
`on_goal_result` already reads `result.error_code` / `error_msg` on ABORTED
(passed to `dump_failure_diagnostics`). Use it to pick the response:
- **planner error** (no valid path / start occupied) -> target unreachable FROM
  HERE -> blacklist point, next.
- **controller / collision error** -> robot cannot move out -> blacklist point +
  wider region, next.
Different blacklist radius per cause.

## Open questions / to decide
- Should a STUCK abandon also trigger a one-shot escape nudge (blind short backup)
  before reselecting, or purely reselect? (Escape recovery is a bug-2 topic; keep
  this experiment to detection+reselection, note the tie-in.)
- Age-out policy for blacklist (time-based? map-growth-based?) to recover from
  early false-negatives without thrashing.
- Metrics to log per abandon: reason (no_frontier / stuck / planner_err /
  controller_err), elapsed, robot_xy, goal_xy, blacklist size — most already in
  telemetry; add a `reason` field for stuck.

## Runs
_(none yet — design notes only)_
