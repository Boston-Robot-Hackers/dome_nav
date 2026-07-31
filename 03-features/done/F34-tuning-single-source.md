# F34 — Tuning Single Source of Truth

**Priority**: High
**Done:** yes (2026-07-30)
**Tasks File Created:** yes (TF34)
**Tests Written:** yes
**Test Passing:** yes (281 passed; 4 live-stack excluded)
**Description**: Explorer tuning is scattered across `FrontierParams`,
`FrontierTuning`, `merge_tuning`, `declare_frontier_params` (every field
hand-transcribed ~4×, `frontier_params.py:11-123`), then repeated again in
3–4 launch files — and split across three override mechanics (node class
constants, node-declared `ExploreParams`, algorithm-declared
`FrontierParams`). Every new knob touches 5–6 sites (F31 added 5 params =
~28 hand edits). This feature makes the dataclass the single source of truth:
declaration, merge, and read-back are driven by `dataclasses.fields()`, the
shared/frontier field ownership is settled and documented, and
`blacklist_radius` becomes a real ROS parameter.

This is the **enabler** for the F32 scoring-pipeline lift and F33 Phase B:
analysis.md Part 6 (`02-doc/analysis.md:349-359`) says the dedup must land
first or in the same task as the lift, because splitting tuning into
scoring-config vs generator-config otherwise adds a fifth transcription
consumer.

## Scope

- `dome_nav/frontier_params.py` — `declare_frontier_params` declare/read via
  `dataclasses.fields()` loop; `merge_tuning` pass-through via `fields()`
  iteration + explicit overlay of the shared `ExploreParams` fields (design
  already sketched in `04-tasks/chores.md` — that chore is absorbed here).
  Keeps the pure dataclass; no ROS leak into `frontier_explorer`/
  `frontier_algorithm`.
- `dome_nav/explorer_manager_node.py` — declare `blacklist_radius` as a ROS
  param wired into `ExploreParams` (fixes the critique-doc bug:
  `ExploreParams.blacklist_radius` is silently pinned to 0.5, not
  yaml/launch-tunable).
- **Settle field ownership** — the analysis.md Part 6 split:
  `preferred_goal_distance` sits in `ExploreParams` while its scorer sits in
  the algorithm; `blacklist_radius` is shared because node reselection uses
  it. Decide which fields are shared vs algorithm-owned, resolve the split
  cleanly (move, don't duplicate), and document the one override mechanic.
- Reconcile launch exposure — audit `robot_explore`, `sim_explore_node`,
  `just_explorer` against the settled param set; update
  `02-doc/tunable_parameters.md` to match reality.

## Non-goals (deliberately excluded, reasoning preserved)

- **Fat-node split** (`explorer_manager_node.py` ~690 lines, ~6
  responsibilities) — the right split depends on where dwell/recovery behavior
  lands, an F33 Phase B + F29 decision. Splitting now means re-splitting
  later.
- **Recovery-policy boundary** (node watchdogs/costmap guards vs Nav2) —
  parked with F29 (deferred 2026-07-29).
- **Deprecated-wire removal** (`prefer_farthest`, `novelty_top_n`,
  `exploration_resume`/`paused_on_failure`, hardcoded `basement1.yaml`) —
  handled as chores; removing them first shrinks the `fields()` surface.

## Constraints

- Pure refactor, no behavior change: merged `FrontierTuning` output must be
  identical on fixtures before/after (except the newly tunable
  `blacklist_radius`, which gains launch override with the same default).
- ROS param types are behavior-sensitive (bool vs int vs float inference from
  dataclass defaults) — the `fields()` loop must map types explicitly.
- Launch-signature repetition stays (better_launch AST-parses literals) —
  documented, not "fixed".

## How to Demo

**Setup**: full pure-test suite runnable (`/usr/bin/python3 -m pytest test/`).

**Steps**:
1. Add a new field to `FrontierParams` — confirm it is declared, readable,
   and merged with zero edits to `declare_frontier_params`/`merge_tuning`.
2. Launch with a `blacklist_radius` override (launch arg or yaml) — confirm
   the node picks it up. Note: `ros2 param set` mid-run will **not** take
   effect; params are read once at construction and nothing is dynamic yet
   (see TF34 T01's `ros_dynamic` flag — intent only).

**Expected output**: adding a tuning knob touches exactly one site (the
dataclass); `blacklist_radius` is launch/yaml-tunable; full suite green.
