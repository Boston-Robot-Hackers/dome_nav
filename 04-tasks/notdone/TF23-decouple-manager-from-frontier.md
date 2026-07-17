# TF23 — Decouple the Explorer Manager from the Frontier Algorithm (tasks for F23)

Sequenced so the suite stays green after each task. T01 is the keystone (it removes
the two biggest leaks); T02/T03 are independent and can follow in either order; T04
is the verification gate; T05 closes out.

## T01 — Intent-carrying result type for `next_goal`
**Status**: done
**Test**: extend `test_frontier_algorithm.py` + `test_hello_world_algorithm.py`:
`next_goal` returns the new result; assert `NEW_GOAL` carries an `(x,y)`,
`EXPLORED_DONE` when the algorithm is finished, `NO_TARGETS_BLOCKED` when clusters
exist but all are filtered. Add a node test that `EXPLORED_DONE` ends the session
and `NO_TARGETS_BLOCKED` triggers the debounce/blacklist-clear path WITHOUT reading
`latest_clusters`.
**Description**: Define a result type in `explore_context.py` — enum or small
dataclass `GoalDecision`: `NEW_GOAL(xy) | NO_TARGETS_BLOCKED | EXPLORED_DONE`.
- Change the `ExplorationAlgorithm.next_goal` signature to return it.
- `FrontierAlgorithm`: return `EXPLORED_DONE` when raw clusters == 0,
  `NO_TARGETS_BLOCKED` when clusters exist but none survive filtering, else
  `NEW_GOAL`. Ownership of the done-condition moves here from the node.
- `HelloWorldAlgorithm`: `NEW_GOAL` first call, `EXPLORED_DONE` after.
- Node `find_and_send_frontier` / `handle_no_frontier`: branch on the result; drop
  the `len(self.algorithm.latest_clusters)` peek (line ~380). Keep the node's
  mechanical policy: patience debounce on `NO_TARGETS_BLOCKED`, blacklist-clear-once,
  `stop_exploring("done")` on `EXPLORED_DONE`. Leave `GOAL_TIMEOUT_S` / `check_stuck`
  untouched (navigation concerns, correctly node-side).

## T02 — Move visualization + diagnostics off the protocol
**Status**: done
**Test**: a node test with a stub algorithm exposing NO clusters publishes markers
without error and writes no-frontier telemetry; assert the protocol no longer
requires `latest_clusters`/`latest_diag`.
**Description**: Remove `latest_clusters`/`latest_diag` as required protocol surface
(`explore_context.py`).
- DECIDE marker ownership (record the decision in this task): either (a) the
  algorithm publishes its own `/explore/markers`, or (b) the algorithm supplies an
  opaque marker/diagnostics payload via an optional method the node renders/logs
  blindly. Recommendation to weigh: (b) keeps one publisher in the node but treats
  the payload as opaque.
- Rework `publish_markers`, `dump_frontier_exhaustion`, `dump_failure_diagnostics`,
  and the `no_frontier` telemetry so they no longer reach into algorithm internals.
- `HelloWorldAlgorithm` drops its faked `latest_clusters=[]` / `latest_diag=None`.

**DECISION — marker ownership: option (b), opaque payload via optional hooks.**
The node keeps its single `/explore/markers` publisher (one QoS, node-owned
lifecycle) and treats the algorithm's contribution as opaque. The
`ExplorationAlgorithm` protocol now requires only `next_goal`; visualization and
diagnostics are OPTIONAL hooks the node calls via `getattr` and never inspects:
- `render_markers(rc: RenderContext) -> MarkerArray | None` — node publishes it
  verbatim; absent hook / `None` → nothing published.
- `exhaustion_report(rc) -> str | None` and `failure_report(rc) -> str | None` —
  node logs the string blindly (failure text is appended to the node-general
  `format_failure_diagnostics` dump via its new `algorithm_report=` arg).
- `telemetry_extra() -> dict` — merged into the `no_frontier` telemetry blindly
  (this is where the old `raw_clusters` / `latest_diag` fields now originate).

`RenderContext` (new, in `explore_context.py`) carries only node-general session
state (now stamp, is_exploring, map_info, robot_xy, blacklist, goal_xy, params) —
no frontier concepts. `FrontierAlgorithm` implements all four hooks and keeps its
own concrete `latest_clusters`/`latest_diag` state for them (no longer protocol
surface; the standalone `tools/algo_demo.py` still reads them off the concrete
class). `HelloWorldAlgorithm` implements none and holds no faked cluster state.
Rationale for (b) over (a): a single publisher avoids duplicate topic ownership,
matched-QoS bookkeeping, and lifecycle races, while opacity keeps the node free of
any frontier knowledge.

Chose (b): one node-owned publisher, algorithm payload treated as opaque.

## T03 — Split params: general vs frontier-owned
**Status**: done
**Test**: unit-test that `FrontierAlgorithm` reads its tuning from `FrontierParams`
(construct with non-defaults, assert behavior); node test that a plugin needing only
the shared params runs without the frontier params declared.
**Description**: Split `ExploreParams`.
- Shared/session set stays general: `max_explore_radius`, `blacklist_radius`,
  `preferred_goal_distance` (revisit `goal_inset_m` — likely frontier).
- New `FrontierParams` (frontier module) owns `min_frontier_size`,
  `min_frontier_dist`, `max_frontier_dist`, `frontier_buffer_cells`, `goal_inset_m`,
  and the deprecated `prefer_farthest`.
- Provide a declaration path so algorithm-owned params are still yaml/launch
  settable: algorithm exposes a param schema the node declares generically, OR the
  node passes a handle so the algorithm self-declares in its namespace. Pick one,
  document why. Node stops declaring frontier ROS params directly.
- Update launch/yaml configs and `robot_explore.launch.py` param passing.

### Resolution (done)

**Split as specified.** `ExploreParams` (in `explore_context.py`) now holds only the
shared/session set: `max_explore_radius`, `blacklist_radius`,
`preferred_goal_distance`. New `FrontierParams` owns `min_frontier_size`,
`min_frontier_dist`, `max_frontier_dist`, `goal_inset_m`, `frontier_buffer_cells`,
and the deprecated `prefer_farthest`. `goal_inset_m` moved to frontier (it is the
frontier-goal nudge, meaningless to a non-frontier strategy).

**Module choice: new `dome_nav/frontier_params.py`** (not `frontier_algorithm.py`).
It defines `FrontierParams`, `declare_frontier_params(node)`, a combined read-only
`FrontierTuning`, and `merge_tuning(shared, frontier)`. A dedicated module avoids a
`frontier_explorer` → `frontier_algorithm` import cycle (the pure functions and the
diagnostics formatters type against `FrontierTuning`, and `frontier_algorithm`
already imports `frontier_explorer`).

**Declaration path: node passes itself as a handle; the algorithm self-declares.**
The `ExplorationAlgorithm` protocol gains an optional `declare_params(node)` hook the
node calls once in `__init__`. `FrontierAlgorithm.declare_params` calls
`declare_frontier_params(self, node)`, which declares each frontier param (by its
original name) in the node's namespace and reads it back into `self.frontier_params`.
Chosen over a node-declared schema because it keeps every frontier param *name* out
of `explorer_manager_node.py` — exactly F23's end goal and T04's grep assertion. A
schema approach would still have the node loop over frontier-derived names/defaults.
`HelloWorldAlgorithm.declare_params` is a no-op (it needs only the shared params), so
a shared-only plugin runs with no frontier params declared.

**Params reach the algorithm two ways, `next_goal(ctx)` unchanged.** Shared params
travel on `ExplorationContext.params` (from the node). Frontier params are owned by
the algorithm (`self.frontier_params`, from `declare_params` or the constructor). Per
tick `FrontierAlgorithm.next_goal` calls `merge_tuning(ctx.params, self.frontier_params)`
to build the combined `FrontierTuning` it feeds to the pure frontier functions. The
deprecated `prefer_farthest → preferred_goal_distance` remap moved out of the node
into `merge_tuning`.

**Still-frontier-coupled node paths (superseded — see note).** At T03 close,
`dump_frontier_exhaustion`, `session_start` telemetry, and `publish_markers` still read
frontier tuning via a node `frontier_tuning()` helper (`merge_tuning(...)` when the
algorithm exposed `frontier_params`, else `None`). **This was removed in a follow-up
param-leak cleanup:** those paths now route through the T02 opaque hooks
(`session_params`, `render_markers`, `exhaustion_report`), so the node no longer reads
`algorithm.frontier_params` or imports `merge_tuning`/`FrontierTuning`.

**Launch/yaml.** Frontier params are still declared by the same names in the node's
namespace, so `robot_explore.launch.py` / `sim_explore*.launch.py` pass them
unchanged (comment updated). No yaml config referenced these params. `blacklist_radius`
and `goal_inset_m` remain unexposed in launch (dataclass defaults).

**`min_frontier_size` default mismatch resolved.** The field moved to `FrontierParams`
(default 15, the dataclass value). `declare_frontier_params` now declares
`min_frontier_size` with default 15, so node default == `FrontierParams().min_frontier_size`
== 15 — consistent. The old node test `test_min_frontier_size_default_matches_explore_params`
(which expected 10) is replaced by `test_frontier_params_defaults_match_dataclass`
against the new `FrontierParams` home. Launch files still override to 10 (real) / 5
(sim), so runtime behavior is unchanged.

**Tests:** `pytest test/ -m "not manual"` → 197 passed. (The 4 `test_map_validation.py`
tests are `pytest.mark.manual` — require a live stack — and are excluded, unchanged by
this task.) New/updated: `test_frontier_algorithm.py` constructs `FrontierAlgorithm`
with non-default `FrontierParams` and asserts `min_frontier_size`, `min_frontier_dist`,
`frontier_buffer_cells`, and `prefer_farthest` change behavior;
`test_explorer_manager_node.py` adds `test_frontier_params_defaults_match_dataclass`,
`test_frontier_params_declared_as_ros_params`, and
`test_shared_only_plugin_declares_no_frontier_params` /
`test_shared_only_plugin_ticks_without_frontier_params`;
`test_frontier_explorer.py` and `test_explore_diagnostics.py` build a `FrontierTuning`
instead of an `ExploreParams`.

## T04 — Verify decoupling (gate)
**Status**: not done
**Test**: `grep -iE 'frontier|cluster|Frontier' dome_nav/explorer_manager_node.py`
returns ONLY the `main()` registry line — add as an assertion / CI-style check where
feasible, else record the grep result here.
**Description**: Audit the node for any residual frontier concept (imports, fields,
comments referencing clusters, param names). Confirm `FrontierAlgorithm` is imported
only for the `main()` registry default. Fix any stragglers.

Known stragglers to address here (params already off the node — post-T03 cleanup):
- **Naming residue (general concepts wearing frontier names):** `NO_FRONTIER_PATIENCE`,
  `no_frontier_count`, the `no_frontier` telemetry key, `find_and_send_frontier`,
  `handle_no_frontier`, `dump_frontier_exhaustion`. These are *no-target* / pick-a-goal
  concepts, not frontier-specific — rename as part of the Explore-vs-Exploration chore.
- **Duplicated `NO_FRONTIER_PATIENCE = 14`:** defined both as a module constant in
  `frontier_algorithm.py` (used only for the `exhaustion_report` label) and as the real
  debounce threshold `ExplorerManagerNode.NO_FRONTIER_PATIENCE`. They match today; if the
  node's value is retuned, the frontier report label drifts. Collapse to one source —
  likely pass the node's value into the report via `RenderContext`, or drop the label's
  dependency on it.
- Confirm the `session_params` / `telemetry_extra` / `render_markers` / `exhaustion_report`
  / `failure_report` opaque hooks leave zero frontier param *names* in the node (T03's
  earlier `frontier_tuning()` helper was replaced by the `session_params` hook, so the
  node no longer reads `algorithm.frontier_params`).

## T05 — Tests, literate, close-out
**Status**: not done
**Test**: full `pytest` green (note the pre-existing `min_frontier_size`
default-mismatch failure separately); harness/robot demo per F23 "How to Demo".
**Description**: Regenerate literate for `explorer_manager_node.py`, `explore_context.py`,
`frontier_algorithm.py`, and `hello_world_algorithm.py` per `.claude/literate.md`.
On completion: move this file to `04-tasks/done/`, set F23 Done/Tests Written/Test
Passing = yes, move F23 to `03-features/done/`.
