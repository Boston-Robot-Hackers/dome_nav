# F29 — Custom BackUp Escape for a Wedged Robot

**Priority**: Medium
**Done:** no
**Tasks File Created:** no
**Tests Written:** no
**Test Passing:** no

> **Number note:** first floated as "candidate F28" in `02-doc/current.md`
> (2026-07-18 wedge section); renumbered to F29 when F28 was claimed for
> reason-tagged goal exclusion (2026-07-20).

**Description**: When the robot is **wedged** — stuck against an obstacle with no
progress — neither Nav2's built-in recovery nor the explorer's reselect loop
extracts it, and exploration stalls. This feature adds an explorer-driven escape: on
detecting no progress, directly command a **BackUp** (reverse a fixed distance away
from the obstacle) *before* reselecting a goal, instead of relying on Nav2's inner
clear-costmap-and-retry loop or waiting out its outer recovery.

## Background (from 2026-07-18 real-robot investigation)

- **Start-wedged case:** the robot begins near an obstacle; the collision_monitor
  `FootprintApproach` polygon gates `cmd_vel` from goal #1, so the robot never moves.
  Reselecting goals cannot fix a wedged **pose** — only the goal changes, not where
  the robot is.
- **Nav2 recovery is the wrong recovery, in the wrong order:** the controller
  `progress_checker` fires `Failed to make progress` (~10s), but the recovery Nav2 runs
  first is `ClearLocalCostmap` + retry `FollowPath` — useless against a real obstacle
  the laser immediately re-marks. `Spin` / `BackUp` live in the **outer** recovery,
  reached only after the inner clear-retry loop exhausts (~2×10s), so within a normal
  stuck window Nav2 never actually backs up. Raising the explorer's `STUCK_T_S` (done
  7→20) only buys more futile clear-costmap loops.
- A `BackUp` is **usually not gated** by the `FootprintApproach` polygon: the approach
  check simulates the *commanded* velocity, and reversing moves the footprint away from
  the scan points, so no collision is found. **But see the source-read caveat below —
  this is not unconditional.**

## Source-read findings (nav2_collision_monitor C++, jazzy 1.3.12, 2026-07-21)

Read of `collision_monitor_node.cpp` / `polygon.cpp` / `kinematics.cpp` upstream:

- **Static-check risk — reverse can be gated too.** `Polygon::getCollisionTime`
  (`polygon.cpp:278`) runs a velocity-blind static check *before* the motion
  simulation: if ≥ `min_points` (ours: 6) scan points lie inside the footprint
  polygon **at the current pose**, it returns `collision_time = 0` and
  `processApproach` scales **all** cmd_vel components to zero — including reverse
  and rotation. The footprint is the `/local_costmap/published_footprint` circle
  (`robot_radius` 0.17; true robot 0.16), so an obstacle surface within 0.17 m of
  base center puts points inside the polygon. Wedged nose-first against the flared
  post base is plausibly exactly this case. **If the static check is active, the
  BackUp escape is dead on arrival** — the F29 probe must test this first, and it
  is *not* optional. Mitigations if gated: `ros2 param set /collision_monitor
  FootprintApproach.enabled false` during the escape (per-polygon `enabled` is
  dynamically settable), or the node's `~/toggle` service (whole-monitor disable).
- **Counter-consideration:** a thin post may return < 6 beams inside the footprint,
  in which case the static check never fires and reverse passes. Point count is
  observable (below) — measure, don't assume.
- **Second stall cause, unrelated to obstacles:** if the scan source fails
  `getData()` — no scan within `source_timeout` 1.0 s **or scan→base TF transform
  fails** — the monitor hard-STOPs everything and logs `"Robot to stop due to
  invalid source"`. The Pi's persistent `laser ... earlier than transform cache`
  TF drops can trigger this, mimicking a wedge with no obstacle. Grep run logs for
  `invalid source` before attributing a stall to wedging.
- **Observability for the probe** (no rebuild needed):
  - `ros2 topic echo /collision_monitor_state` — `{polygon_name, action_type}` on
    action change; distinguishes `FootprintApproach` gating from `invalid source`.
  - `ros2 topic echo /collision_monitor/collision_points_marker` — the exact scan
    points in base frame (lazy: publishes only while subscribed). Count points
    within the 0.17 m footprint ⇒ static-check active or not.
  - Ratio `cmd_vel.linear.x / cmd_vel_smoothed.linear.x` recovers the throttle
    factor `collision_time / time_before_collision` (not published anywhere).

## Idea

On the explorer's no-progress detection (the existing stuck watchdog), instead of only
cancelling + blacklisting + reselecting:

1. Directly invoke a **BackUp** behavior (reverse a small fixed distance, e.g. 0.2–0.3 m,
   at low speed) via the Nav2 `BackUp` action / behavior server — bypassing the
   clear-costmap dance entirely.
2. Only after the BackUp completes, reselect a goal and resume.
3. Cap consecutive escapes so a truly trapped robot fails cleanly instead of reversing
   forever.

**Mandatory probe first** (upgraded from optional after the 2026-07-21 source read —
see "Source-read findings"): confirm on the real robot that a direct `BackUp` frees a
wedged robot at all, watching `collision_points_marker` for the static-check condition
(≥ min_points scan points inside the footprint ⇒ reverse is gated too). If the probe
shows BackUp does not free it, this feature's approach is invalidated and the escape
needs the `FootprintApproach.enabled false` toggle (or a different fix).

## Scope (in)

- Explorer-side escape: on stuck (no-progress), command a `BackUp` before reselecting.
- Consecutive-escape cap + clean failure when the cap is hit.
- Escape events surfaced in `/explore/status` + telemetry (so a wedge-and-escape is
  visible, not hidden inside a reselect).

## Scope (out)

- **Detecting** the wedge via footprint/local-costmap analysis (start-pose wedge
  detection) — this feature reuses the *existing* no-progress stuck signal as the
  trigger; footprint-based detection is a separate concern (see F27 scope-out).
- Reason-tagged goal exclusion — that is **F28**.
- Any change to collision_monitor / `FootprintApproach` configuration.

## Constraints

- BackUp must not itself drive the robot into a *rear* obstacle — bound the distance
  and, if feasible, gate on the rear being clear.
- Escape is a **navigation/node-owned** behavior (like the stuck watchdog), not an
  exploration-algorithm concern — keep it out of the F23-decoupled algorithm.
- Must degrade cleanly if the `BackUp` action/behavior server is unavailable (log +
  fall back to today's cancel-and-reselect).
- Real-robot-only validation: the wedge reproduces on hardware (start near a post);
  sim may not exercise the collision_monitor gating identically.

## How to Demo

**Setup**: real robot, Mode E explore, started deliberately nose-first near an
obstacle (the start-wedged condition). Base stack + `robot_explore` (or the
`nav_experiment` harness) running; `ros2 topic echo /explore/status`.

**Steps**:
1. Start exploration with the robot wedged against a post/wall so goal #1 is gated by
   `FootprintApproach` and the robot cannot move forward.
2. Wait for the no-progress stuck detection to fire.
3. Observe the explorer command a `BackUp`.

**Expected output**: the robot **reverses** a fixed distance away from the obstacle,
the collision_monitor gate releases, and exploration resumes with a fresh goal —
instead of stalling in Nav2's clear-costmap-and-retry loop until patience exhaustion.
The escape (and any cap-hit clean failure) appears in `/explore/status` + telemetry.
