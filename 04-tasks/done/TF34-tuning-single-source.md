# TF34 — Tuning Single Source of Truth for F34

Order matters: the deprecated-wire chores land **first** (they shrink the
`fields()` surface), then T01–T05.

## T01 — `fields()`-driven declare/read + merge dedup
**Status**: done (2026-07-30)
**Description**: Replace the hand-listing in `declare_frontier_params`
(`frontier_params.py:90-123`) with a `dataclasses.fields()` loop: declare each
field name/default, then construct `FrontierParams(**{f.name:
node.get_parameter(f.name).value ...})`. Map dataclass types to ROS param
types explicitly (bool/int/float — inference from default value type is fine
but must be deliberate, not accidental; e.g. `max_frontier_dist: float = 0.0`
infers double, `min_frontier_size: int = 15` infers integer). Reduce
`merge_tuning`'s pass-through to a `fields()` iteration plus an explicit
overlay of the shared `ExploreParams` fields; keep the
`blacklist_radius > goal_inset_m` boundary validation. (The
deprecated-`prefer_farthest` mapping was deleted with the field's removal on
2026-07-30.)

End-state note (so T01 and T03 don't diverge): with the `prefer_farthest`
removal landed (2026-07-30) and T03's planned move of
`preferred_goal_distance` into `FrontierParams`,
the shared overlay is exactly **2 fields** (`max_explore_radius`,
`blacklist_radius`) — not the "3 shared fields" the older design notes say.
T01 should implement the overlay generically enough that T03's field move
doesn't require restructuring it.

Each field carries its declaration as dataclass metadata, so the description
lives at the single source of truth:

```python
min_frontier_size: int = field(default=15, metadata={
    "ros_description": "Minimum cells in a frontier cluster to be a candidate",
    "ros_important": True,
    "ros_dynamic": False,
})
```

- `ros_description` (str) → `ParameterDescriptor(description=...)`, so
  `ros2 param describe` explains the knob; units stated explicitly (m, cells).
- `ros_important` (bool) — **documentation only for now**: marks the knobs an
  operator should pay attention to when tuning, vs structural/rarely-touched
  fields. Nothing reads it in code; intended consumers later are docs
  generation (`tunable_parameters.md`) and launch/tuning tooling. First-pass
  marking (adjust freely):
  - important: `min_frontier_size`, `min_frontier_dist`, `max_frontier_dist`,
    `goal_inset_m`, `w_clearance`, `robot_radius`, `clearance_margin_m`
    (+ `preferred_goal_distance` when T03 moves it in)
  - not important: `frontier_buffer_cells`, `use_novelty_scoring`,
    `w_distance`, `w_novelty`
- `ros_dynamic` (bool) — **documentation only for now** (same status as
  `ros_important`): marks whether the knob is *intended* to be adjustable
  while running (`ros2 param set` mid-session takes effect) vs read-once
  startup configuration. Nothing reads it in code, and today nothing is
  actually dynamic — `declare_frontier_params` reads once at construction.
  The flag records design intent for tuning tooling and for the future task
  that wires a parameter-event callback to honor it. Marking:
  - dynamic: `w_distance`, `w_novelty`, `w_clearance`, `use_novelty_scoring`
    (+ `preferred_goal_distance` when T03 moves it in) — the levers the
    `just_explorer` tuning harness wants to sweep without relaunching
  - static: everything else (`robot_radius`, `min_frontier_size`, buffer,
    inset, dist bounds) — changing them mid-run would invalidate per-tick
    invariants anyway
- The three scorer weights also declare `FloatingPointRange(from_value=0.0)`
  so negative weights are rejected by ROS itself (range carried in metadata
  too, e.g. `"ros_min": 0.0`).
**Test**: fake node round-trips every field through declare+read and gets the
defaults back; every field carries non-empty `ros_description` and bool
`ros_important`/`ros_dynamic` in its metadata (doc surface can't silently
rot); weights reject negative values via the declared range; `merge_tuning`
output is field-for-field identical to the
pre-refactor output on fixed fixtures (including override cases); adding a
temporary field requires zero edits outside the dataclass (prove it, then
remove the temp field).

## T02 — Declare `blacklist_radius` as a ROS param
**Status**: done (2026-07-30)
**Description**: `explorer_manager_node.py` declares `blacklist_radius`
(default 0.5, matching `ExploreParams`) and wires it into `ExploreParams` —
currently undeclared, so it's silently pinned to 0.5 (critique-doc bug #2,
`02-doc/architecture_critique_explore.md:249`). Expose in the launch files
that already expose the other shared params.
**Test**: node constructed with a parameter override produces
`ExploreParams.blacklist_radius == override`; default unchanged at 0.5.
Regression: `merge_tuning` validation still fires on `radius <= goal_inset_m`
when the override is out of bounds.

## T03 — Settle shared vs frontier field ownership
**Status**: done (2026-07-30)
**Decision**: ownership rule = *shared (`ExploreParams`) iff the node itself
reads it for its own policy* (radius gating / blacklist reselection); a telemetry
echo does not count. `preferred_goal_distance` (scorer-only) moved to
`FrontierParams`; `HelloWorldAlgorithm` — the only other reader — gained its own
same-named `preferred_goal_distance` step param (declared in its `declare_params`).
Shared overlay is now exactly 2 fields (`max_explore_radius`, `blacklist_radius`).
Node telemetry key `preferred_goal_distance` preserved via
`FrontierAlgorithm.session_params()` (was `session_start_params`). Consumers
updated: `frontier_params.py`, `explore_context.py`, `frontier_algorithm.py`,
`explorer_manager_node.py`, `hello_world_algorithm.py`, tests, and the doc rule.
**Description**: Resolve the analysis.md Part 6 split:
`preferred_goal_distance` lives in `ExploreParams` (shared) but only its
scorer (algorithm-side) uses it; `blacklist_radius` is shared because node
reselection filters against it. The ownership rule is "shared iff the node
itself reads it" — with one wrinkle: `hello_world_algorithm.py:27` also reads
`ctx.params.preferred_goal_distance` (as its step distance) and has no
`FrontierParams`, so the rule must be stated as "shared iff the node *or a
second algorithm* reads it", or HelloWorld gets its own param. Decide, then
move fields to match — likely `preferred_goal_distance` → `FrontierParams` —
update every consumer (node telemetry at `explorer_manager_node.py:214`,
`merge_tuning`, **all five** launch files — see T04 — `tools/algo_demo.py`,
`hello_world_algorithm.py` if it loses the shared read), and write the
rule + the one override mechanic into `02-doc/tunable_parameters.md` and the
`frontier_params.py` module docstring.

Groundwork already landed (2026-07-30): `ExploreParams` carries `tuning_field()`
metadata and the node declares it via `declare_dataclass_params(self,
ExploreParams)` — the generic machinery lives in `explore_context.py`, shared
by both dataclasses, so T03 starts from uniform declaration on both sides.
**Test**: consumers updated, full suite green; a test asserts the node's
`session_params` telemetry still reports the moved fields (wire-contract
stability — telemetry key names must not change even if the owning dataclass
does).

## T04 — Launch-exposure reconciliation
**Status**: done (2026-07-30)
**Result**: the `preferred_goal_distance` move is transparent to launch — all
five files still set it by the same name, now declared by `FrontierAlgorithm` in
the node namespace instead of the node's `ExploreParams`. No launch file sets a
removed/renamed param; all five byte-compile. `blacklist_radius` exposed in
`robot_explore`/`sim_explore_node`/`just_explorer`/`nav_experiment`;
`sim_nav_full` intentionally leaves it default. `tunable_parameters.md` exposure
matrix, §1/§2 tables, §7 quirks updated to match.
**Description**: Audit **all five** launch files that set explorer params —
`robot_explore.launch.py`, `sim_explore_node.launch.py`,
`just_explorer.launch.py`, plus `sim_nav_full.launch.py:76-80` and
`nav_experiment.launch.py:33,77-82` (these last two are easy to miss; a
moved/removed param makes them fail loudly at launch) — against the settled
param set from T03: every
intentionally-tunable param exposed, no launch file setting a moved/removed
param, sim-vs-real value deltas intentional and commented. Update
`02-doc/tunable_parameters.md`'s exposure matrix to match the final state.
**Test**: `colcon build` clean; launch files parse (`bl --show-args` or
equivalent dry inspection per file); manual diff of tunable_parameters.md
matrix against the launch sources.

## T05 — Full verification + literate + docs
**Status**: done (2026-07-30)
**Result**: full suite green (281 passed; 4 live-stack `test_map_validation`
need a robot). `colcon build` clean. Literate regenerated: `07-frontier_params.md`
(two-field overlay + ownership rule), `08-frontier_algorithm.md` (stale
hand-listed declare/merge replaced with the `fields()`-driven form + pointer to
07). Promoted DRY-param chore deleted from `chores.md`. F34/TF34 flags flipped
and moved to `done/`.
**Description**: Full suite (`/usr/bin/python3 -m pytest test/`) green;
regenerate literate for `frontier_params.py`, `explorer_manager_node.py`, and
any other touched module (includes the stale plumbing section of
`01-literate/08-frontier_algorithm.md`, which still shows the pre-T01
hand-listed code); update `02-doc/current.md` (F34 done, chores
absorbed); delete the promoted DRY-param entry from `04-tasks/chores.md`
(marked "remove this entry when TF34 lands"); flip F34 flags and move
F34/TF34 to `done/`.
**Test**: suite green is the test; spot-check one literate regen against its
source module.
