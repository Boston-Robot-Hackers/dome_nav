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
**Status**: not done
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

## T03 — Split params: general vs frontier-owned
**Status**: not done
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

## T04 — Verify decoupling (gate)
**Status**: not done
**Test**: `grep -iE 'frontier|cluster|Frontier' dome_nav/explorer_manager_node.py`
returns ONLY the `main()` registry line — add as an assertion / CI-style check where
feasible, else record the grep result here.
**Description**: Audit the node for any residual frontier concept (imports, fields,
comments referencing clusters, param names). Confirm `FrontierAlgorithm` is imported
only for the `main()` registry default. Fix any stragglers.

## T05 — Tests, literate, close-out
**Status**: not done
**Test**: full `pytest` green (note the pre-existing `min_frontier_size`
default-mismatch failure separately); harness/robot demo per F23 "How to Demo".
**Description**: Regenerate literate for `explorer_manager_node.py`, `explore_context.py`,
`frontier_algorithm.py`, and `hello_world_algorithm.py` per `.claude/literate.md`.
On completion: move this file to `04-tasks/done/`, set F23 Done/Tests Written/Test
Passing = yes, move F23 to `03-features/done/`.
