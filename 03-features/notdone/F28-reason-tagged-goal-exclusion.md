# F28 — Reason-Tagged Goal Exclusion with Decaying Transient Rejects

**Priority**: Medium
**Done:** no
**Tasks File Created:** no
**Tests Written:** no
**Test Passing:** no

> **Number note:** `02-doc/current.md` (2026-07-18 wedge section) tentatively called
> the custom BackUp-escape idea "candidate F28", but no file was created. F28 is
> claimed here for reason-tagged exclusion; the BackUp escape moves to **F29**.

**Description**: The explorer currently excludes goals through **two disjoint,
reason-less mechanisms**, and neither records *why* a cell was excluded:

- **Persistent `blacklist`** (`set[XY]`, `blacklist_radius`): populated by
  navigation failures — stuck, timeout, aborted/rejected. Cleared once on patience
  exhaustion.
- **Per-tick `rejected`** (`set[XY]`, no radius): populated by the F27 goal guards —
  out-of-bounds (`goal_within_costmap_bounds`) and lethal/inscribed
  (`goal_is_lethal`). Discarded every tick.

Two problems follow:

1. **Silent re-reject loop.** Because `rejected` resets each tick, an algorithm that
   keeps returning the *same* out-of-bounds or lethal cell is rejected forever with no
   progress, no record, and no visible reason — the robot stalls into
   `NO_TARGETS_BLOCKED` patience exhaustion while the frontier still looks selectable.

2. **Diagnostics can't explain the stall.** `format_frontier_exhaustion` labels a
   cluster `OK` whenever it passes the algorithm's *geometric* filters (size, distance,
   persistent blacklist). It has no knowledge of the node-side guards, so a cluster the
   node rejects every tick still prints `OK`. Operators see "frontier exhaustion, but
   many centroids are OK" with no way to tell why.

This feature unifies exclusion into **one reason-tagged store** and gives each
exclusion reason its own lifetime, so the dump can report the true reason and the
transient rejects decay instead of looping silently or sticking forever.

## Motivation (from 2026-07-20 review)

- Bounds and lethal rejects are properties of the **costmap at this instant**, not of
  the goal: the global costmap grows as the robot moves / SLAM expands, and lethal
  cells clear when inflation updates. Persistently blacklisting them would permanently
  kill frontiers that later open up — so they must **not** become sticky blacklist.
- But per-tick discard is the opposite failure: no memory at all, so a repeated reject
  is invisible and unbreakable within a session.
- The right shape is a **reason-tagged store with per-reason lifetime**: nav-failure
  reasons stay until patience-exhaustion clear (today's behavior); transient
  costmap reasons (oob/lethal) get a short TTL / decay and clear on costmap update.

## Scope (in)

- **Reason-tagged exclusion store** replacing the bare `blacklist: set[XY]` and the
  per-tick `rejected: set[XY]`. Each excluded cell carries a reason: `nav_abort`,
  `stuck`, `timeout`, `oob`, `lethal` (names settled in the task list). Radius
  membership (`blacklist_radius`) preserved for the nav-failure reasons.
- **Per-reason lifetime policy**:
  - nav-failure reasons: sticky until the existing patience-exhaustion clear.
  - transient reasons (oob/lethal): short TTL / decay, and cleared when a fresh
    global costmap arrives (the state that caused them may be gone).
- **Repeated-reject escalation**: a cell rejected as oob/lethal for N consecutive
  ticks escalates to a bounded-TTL exclusion so the same cell is not re-selected every
  tick — breaking the silent loop while preserving retry-after-costmap-change.
- **Reason surfaced in diagnostics**: `format_frontier_exhaustion` reports the
  exclusion reason per cluster/cell — an `OK` line becomes e.g.
  `OK-but-rejected(lethal)`. `/explore/status` and telemetry expose the reason
  breakdown (counts per reason).

## Scope (out)

- The **custom BackUp escape** for a start-wedged robot — that is **F29**, not this
  feature. F28 is about *which goals are excluded and why*, not about extracting a
  physically wedged robot.
- **Giving the algorithm costmap awareness** (passing the global costmap into
  `ExplorationContext` so selection avoids lethal/oob cells up front). That would move
  the guards inside the algorithm and largely subsume the reject store; it is a larger
  F23-protocol change, deferred (see F27 scope-out). F28 keeps the node-side guard
  split intact and only makes exclusion legible and correctly-lived.
- Changing the F27 guard bands or the costmap scale — unchanged here.

## Constraints

- Transient (oob/lethal) exclusions must never become permanent: a frontier that
  becomes valid after the costmap grows/clears must be re-selectable.
- Node-side guard split stays (algorithm blind to costmaps) unless the scope-out
  "costmap awareness" item is later adopted.
- One exclusion store; no re-introduction of a second disjoint reason-less set.
- Behavior-preserving for the nav-failure path: same stickiness and same
  patience-exhaustion clear as today, now just reason-tagged.

## How to Demo

**Setup**: sim stack running
(`bl dome_nav sim_nav_full.launch.py --map_name f28test --world_name multi_room`),
`ros2 topic echo /explore/status`.

**Steps**:
1. Drive exploration so a frontier's best candidate repeatedly lands on a lethal or
   out-of-bounds cell near a wall / at the costmap edge.
2. Trigger the exhaustion dump (let the robot reach patience exhaustion, or inspect
   the periodic dump).
3. Read the exhaustion dump lines and `/explore/status`.

**Expected output**: the offending cluster is reported with its real reason
(`OK-but-rejected(lethal)` / `oob`), not a bare `OK`; the same cell is not
re-selected every tick (escalated to a bounded-TTL exclusion); once the costmap grows
or clears, the cell becomes selectable again (transient exclusion did not stick).
