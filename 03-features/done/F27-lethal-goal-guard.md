# F27 — Never Dispatch a Goal to a Lethal Location

**Priority**: High
**Done:** yes (2026-07-29 — code + unit tests done; T06 sim / T07 live marked done, but note both are very hard to really verify: can't force a nudged goal onto a lethal cell on demand. Confidence rests on unit tests + live-observed behavior, not a staged repro.)
**Tasks File Created:** yes (TF27)
**Tests Written:** yes (unit)
**Test Passing:** yes (229 passed, 4 deselected)
**Description**: Ensure the explorer never asks Nav2 to travel to a goal whose
global-costmap cell is lethal. Today `goal_in_global_costmap()` in
`explorer_manager_node.py` is a **bounds-only** guard: it passes any goal inside
the costmap rectangle regardless of cost, so a frontier goal — especially after
`nudge_toward_robot()` pulls it inward — can land on a lethal cell and still be
dispatched. Nav2 then rejects it (`GOAL in collision` / `NO_VALID_PATH`), wasting
a goal cycle. This feature adds a lethal-cost check to the goal guard, applied to
the **final (post-nudge)** goal, and reconciles the costmap cost encoding across
the node and its diagnostics so both read one scale.

## Background (from 2026-07-18 investigation)

- The explore algorithm is **blind to costmaps** — `ExplorationContext` carries only
  the SLAM `/map` occupancy grid, `map_info`, `robot_xy`, `blacklist`, `start_xy`,
  `params`. Goal selection, novelty scoring, and the nudge all run on `/map`, which
  has no inflation or lethal-cost information.
- The node fetches `/global_costmap/costmap` and `/local_costmap/costmap` but keeps
  them to itself: the global one is used **only** for the bounds check; the local one
  **only** for failure-diagnostics logging. Neither reaches goal selection.
- **Encoding trap:** the node reads `/global_costmap/costmap` = `nav_msgs/OccupancyGrid`,
  which is a **scaled** grid (lethal `100`, not the raw `254`), and `explore_diagnostics.py`
  assumes the raw 0–255 scale — so its `== 254` check never fires. Node and diagnostics
  must be reconciled onto one scale. The exact translation table, threshold options, and
  the recommended choice live in **TF27 T01**.

## Scope (in) — detail in TF27

- **Lethal-goal guard**: reject the **final post-nudge** goal when its global-costmap
  cell is lethal (recommended band: lethal + inscribed). Reuse the existing skip-and-
  reselect loop; degrade permissively when no costmap has arrived yet. (TF27 T02)
- **One cost scale** across the node and `explore_diagnostics.py`, fixing the
  `== 254`-never-fires bug. (TF27 T01)
- **Rename** the bounds guard `goal_in_global_costmap` → `goal_within_costmap_bounds`
  (bounds-only meaning made honest); lethal check is a separate guard. **Done** as a
  chore 2026-07-18. (TF27 T03)

## Scope (out — related ideas captured, not built here)

These came up in the same discussion and are valid, but are **not** the lethal-goal
requirement. Listed so they are visible in one place; each is a separate feature if
pursued.

- **Proven-obstacle check on `/map`** (`data == 100`): cheap cross-check for goals on
  observed walls, useful when the global costmap window lags or is smaller than `/map`.
  Largely overlaps the global-costmap check (global already contains `/map` obstacles +
  inflation), so low marginal value; deferred.
- **Soft-avoid / inflation-aware goal ranking**: reject or down-rank goals on high
  inflation (`1–98` scaled / `1–252` raw), not just lethal. Behavior change to
  selection quality, not a safety guard — separate feature.
- **Start-pose wedge detection (local costmap + footprint):** the near-wall
  "robot won't start moving" stall is a **current-pose** problem, and the local
  costmap (5×5 m rolling window, ±2.5 m) is the right grid for it — but frontier goals
  are usually **outside** that window, so the local costmap cannot validate a distant
  goal. Footprint validity is a footprint-polygon convolution over cells, not a single
  cell-cost read. Distinct feature: detect/report a start-wedged robot.
- **Give the algorithm costmap awareness**: pass `latest_global_costmap` into
  `ExplorationContext` so selection avoids lethal cells up front instead of the node
  filtering afterward. Larger change to the F23 protocol/decoupling; deferred.

## Constraints

- Node-side guard only; the F23 decoupling (algorithm blind to costmaps) stays intact
  unless the "give the algorithm costmap awareness" out-of-scope item is later adopted.
- One cost scale across node + diagnostics after this feature — no mixed 0–100 / 0–255.
- No change to the startup-permissive default (missing costmap ⇒ allow).

## How to Demo

**Setup**: sim stack running
(`bl dome_nav sim_nav_full.launch.py --map_name f27test --world_name multi_room`).

**Steps**:
1. `ros2 topic echo /explore/status`
2. Drive exploration near a wall/obstacle so a nudged frontier goal would fall on a
   lethal cell.
3. Watch node logs: the lethal candidate is skipped ("goal on lethal cell — skipping
   to next candidate") and a different goal is dispatched, instead of a Nav2
   `GOAL in collision` / `NO_VALID_PATH` abort.

**Expected output**: no goals dispatched onto lethal global-costmap cells; no
`GOAL in collision` aborts caused by lethal goals; node and diagnostics report the
same cost scale.
