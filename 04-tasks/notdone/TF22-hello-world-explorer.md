# TF22 — Hello-World Minimal Explorer Plugin (tasks for F22)

Do T01 first; its findings may revise T02/T03 before any code is written.

## T01 — Critique the pluggable (F12) architecture
**Status**: done
**Test**: not feasible — analysis/design task. Deliverable is a written critique
section appended to this file (## Critique, below), not code. Any concrete fix it
yields becomes a new task step, a chore, or an issue, each of which carries its own
test.
**Description**: Audit the F12 seam using the hello-world plugin as the forcing
function (the smallest possible plugin exposes interface leaks). Judge each:

- **Misplaced logic — manager vs algorithm** (`explorer_manager_node.py`):
  - `goal_in_global_costmap` / `MAX_GOAL_ATTEMPTS` reject-and-retry loop in
    `find_and_send_frontier`: is "is this goal reachable" an algorithm concern the
    node currently second-guesses?
  - `nudge_toward_robot` / `goal_inset_m`: goal post-processing lives in the
    algorithm — right place, or a node/navigator concern?
  - blacklist ownership: node mutates `self.blacklist`; algorithm only reads
    `ctx.blacklist`. Split coherent?
  - `handle_no_frontier` patience/done logic reaching into
    `algorithm.latest_clusters` to make the done call — leaky?
- **Interface completeness**: `next_goal` returns `(x,y) | None`; `None` conflates
  "nothing this tick" with "fully explored," forcing the node to peek at
  `latest_clusters`. Should the return type carry that intent (enum / result
  object)?
- **`latest_clusters` / `latest_diag` side-channel**: exist only for the node's
  markers + telemetry; frontier-specific fields on a general protocol. Hello-world
  must fake `[]`. Is this the right seam?
- **Naming parallelism**: Explore vs Exploration inconsistency across
  `ExplorationContext` / `ExplorationAlgorithm` / `ExploreParams` /
  `ExplorerManagerNode`; `FrontierAlgorithm` vs `find_and_send_frontier`.
- **Selection seam**: confirm constructor-only injection (`main()` at
  `explorer_manager_node.py:598`) is the whole runtime-selection gap.

Deliverable: ranked (value/effort) recommendation list. Route each to: fold into
T02/T03 scope, file as chore (`04-tasks/chores.md`), or raise as issue
(`05-issues/open/`).

### Critique (findings)

Evidence read: `explore_context.py`, `frontier_algorithm.py`, `frontier_explorer.py`,
`explorer_manager_node.py`. Judged each boundary question:

**1. `latest_clusters` / `latest_diag` are a frontier-specific side-channel on a
general protocol.** [LEAK — biggest] The `ExplorationAlgorithm` protocol declares
`latest_clusters: list[list[int]]` + `latest_diag: dict | None`. The node reads
them in SIX places: `dump_frontier_exhaustion` (168), `dump_failure_diagnostics`
(180), `handle_no_frontier` done-decision (371, 380), `publish_markers` (562). Only
`FrontierAlgorithm` naturally has clusters; hello-world must fake `[]`/`None`. A
general seam should not hang frontier internals on every plugin. The node renders
and reasons about an algorithm's private structure.

**2. `next_goal -> (x,y) | None` conflates two distinct outcomes.** [LEAK — root
cause of #1] `None` means BOTH "no target this tick (blocked/filtered)" AND "fully
explored (done)." The node cannot tell them apart from the return, so it recovers
intent by peeking at `len(self.algorithm.latest_clusters)` (line 380: `raw > 0` ⇒
blocked-not-done, clear blacklist once; `raw == 0` ⇒ truly done). That peek is only
necessary BECAUSE the return type is lossy. Fixing the return type (result object /
enum: `NEW_GOAL | NO_TARGETS_BLOCKED | EXPLORED_DONE`) removes the node's reach into
cluster internals — #1 and #2 collapse into one fix.

**3. `goal_in_global_costmap` / `MAX_GOAL_ATTEMPTS` reject-retry loop lives in the
node.** [ACCEPTABLE — with caveat] `find_and_send_frontier` (334-353) asks the
algorithm, rejects any candidate outside the global costmap, and re-asks up to N
times, threading a local `rejected` set into `ctx.blacklist | rejected`. This is
navigator-feasibility filtering against a node-owned resource (global costmap not in
`ctx`), so it is legitimately node-side. Smell: re-invoking `next_goal` N times per
tick and reusing the blacklist channel to exclude candidates conflates "failed
permanently" with "infeasible right now." Minor; leave, but note.

**4. blacklist ownership: node mutates, algorithm reads via `ctx`.** [COHERENT —
keep] Node owns `self.blacklist` (adds on stuck/timeout/planner+controller failure:
286, 309, 460, 505; clears once in `handle_no_frontier`). Algorithm reads
`ctx.blacklist` read-only. Failure memory is session state = a node concern; the
algorithm stays a near-pure decision function. Split is right.

**5. `nudge_toward_robot` / `goal_inset_m` goal post-processing in the algorithm.**
[CORRECT — keep] Pulling the goal a fixed inset off the unknown edge is
goal-SHAPING, a frontier-specific concern. Right side of the boundary; hello-world
simply won't use it.

**6. Naming: Explore vs Exploration is inconsistent across the seam.** [MINOR —
chore] `ExploreParams`, `ExplorerManagerNode`, `explore_tick`,
`explore_algorithm` use "Explore"; `ExplorationContext`, `ExplorationAlgorithm` use
"Exploration." Also `Pluggable` in the node name is now redundant (pluggability is
the whole design). Pure churn to fix (touches tests/imports); no behavior change.

### Recommendations (ranked value/effort)

| # | Recommendation | Value | Effort | Route |
|---|----------------|-------|--------|-------|
| R1 | Replace `next_goal` return with an intent-carrying result (`NEW_GOAL / NO_TARGETS_BLOCKED / EXPLORED_DONE`); move markers/diag off the protocol to an opaque optional `diagnostics()`. Kills findings #1+#2. | High | Med (touches protocol + FrontierAlgorithm + node + tests) | **Issue I12 → feature F23** — too big to fold into F22 without ballooning scope |
| R2 | Rename for parallelism: settle on "Explore*"; drop redundant `Pluggable`. | Low | Low-Med (mechanical, wide) | **Chore** |
| R3 | Document `goal_in_global_costmap` retry loop as a navigator-feasibility gate (not permanent blacklist); consider a separate exclusion channel. | Low | Low | Note only (no action now) |

**Decision for F22:** proceed on the CURRENT interface. Hello-world works by faking
`latest_clusters=[]` / `latest_diag=None` and using `None`-return + patience for
"done." That very fakery IS the evidence backing I12 — do not fold R1 into F22
(scope balloon); T02/T03 unchanged.

## T02 — Implement `HelloWorldAlgorithm`
**Status**: done
**Test**: `test/test_hello_world_algorithm.py` (see T04) — first `next_goal` returns
a goal ~`preferred_goal_distance` ahead in map +x; second call returns `None`.
**Description**: New `dome_nav/hello_world_algorithm.py`. Full `ExplorationAlgorithm`
protocol, least code that still drives:
- `next_goal(ctx)` ignores the map; emits ONE goal `ctx.params.preferred_goal_distance`
  metres ahead of the robot in the map +x direction, then `None` on every later call
  (declares done).
- Protocol attributes the node reads: `latest_clusters = []`, `latest_diag = None`
  (markers + no_frontier telemetry keep working with zero clusters).
- Pure Python, no ROS, no map parsing (~15 lines). Heavily commented as the
  reference template: what the protocol requires, what `ctx` gives, why `None`=done.
- Adjust per any T01 interface change adopted into F22.

## T03 — Runtime selection mechanism
**Status**: not done
**Test**: unit-test the registry lookup helper (name→class, unknown→fallback) with
no ROS spin; extract the mapping into a pure function so it is testable.
**Description**: Edit `main()` in `explorer_manager_node.py`:
- Declare ROS param `explore_algorithm` (string, default `"frontier"`).
- Registry `{"frontier": FrontierAlgorithm, "hello": HelloWorldAlgorithm}`; inject
  the chosen instance into `ExplorerManagerNode(algorithm=...)`.
- Unknown name → warn + fall back to `"frontier"`. Default launch byte-for-byte
  unchanged; production launches untouched.

## T04 — Unit tests (dedicated test task)
**Status**: not done
**Test**: this IS the test task. `pytest` green, zero ROS deps.
**Description**: New `test/test_hello_world_algorithm.py`:
- first goal ~`preferred_goal_distance` ahead in +x from a given `robot_xy`;
- second call returns `None` (done);
- honors `preferred_goal_distance` from `ExploreParams`.
Plus the registry-lookup test from T03. Confirm the existing
`test_frontier_algorithm.py` still passes (no regression from T03 main() edit).

## T05 — Literate docs + on-robot demo
**Status**: not done
**Test**: run the F22 "How to Demo" steps on hardware/harness; `explore_algorithm:=hello`
sends exactly one step goal then declares done; default launch = old frontier
behavior.
**Description**: Regenerate literate doc for `hello_world_algorithm.py` (and the node
if `main()` changed) per `.claude/literate.md`. Run tests + demo before commit.
On completion: move this file to `04-tasks/done/`, set F22 Done/Tests Written/Test
Passing = yes, move F22 to `03-features/done/`.
