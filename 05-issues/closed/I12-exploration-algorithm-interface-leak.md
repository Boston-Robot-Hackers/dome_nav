# I12 — ExplorationAlgorithm interface leaks frontier internals to the node

**CLOSED — converted to feature F23** (`03-features/notdone/F23-decouple-manager-from-frontier.md`,
tasks `04-tasks/notdone/TF23-*`). Retained as the backing critique.

**Goal:** make `explorer_manager_node.py` as independent of `FrontierAlgorithm`
(and frontier concepts generally) as possible. The node should own ROS, Nav2, and
the exploration *session*; the algorithm should own the decision AND its own
frontier-specific knowledge. Today the manager is threaded through with frontier
assumptions on three fronts: its outputs, its done-logic, and its inputs.

* **Symptom.** The F12 pluggable seam forces the manager node to reach into
  algorithm-private structure, and hardcodes frontier semantics the node should
  not know.

  **(A) Output side — lossy return + frontier side-channel.**
  1. `next_goal(ctx) -> (x,y) | None` conflates "no target this tick (blocked)"
     with "fully explored (done)." The node cannot distinguish them from the
     return, so `handle_no_frontier` peeks at `len(self.algorithm.latest_clusters)`
     (`explorer_manager_node.py:380`) to decide blocked-vs-done.
  2. The `ExplorationAlgorithm` protocol (`explore_context.py:38`) declares
     frontier-specific `latest_clusters` + `latest_diag`. The node reads them in six
     places (dump_frontier_exhaustion:168, dump_failure_diagnostics:180,
     handle_no_frontier:371/380, publish_markers:562). Any non-frontier plugin
     (e.g. the F22 hello-world) must fake `latest_clusters=[]`, `latest_diag=None`.

  **(B) Done-decision is algorithm-specific logic living in the node.**
  `handle_no_frontier` hardcodes a frontier-shaped done-rule: `NO_FRONTIER_PATIENCE`
  (14) empty ticks AND `raw_clusters == 0`. "When am I done" depends entirely on
  what `None` means for the algorithm — a random-walk never returns `None` (never
  "done" by this rule); hello-world knows it is done after one goal but is forced to
  wait 14 ticks. Only the *debounce* ("don't panic on a few empty ticks") is
  legitimate node policy; the done-condition belongs to the algorithm. NOTE: goal
  timeout (`GOAL_TIMEOUT_S`) and stuck detection (`check_stuck`) are NOT
  algorithm-specific — they react to navigation execution and correctly stay in the
  node.

  **(C) Input side — flat `ExploreParams` mixes general and frontier tuning.**
  `ExploreParams` (`explore_context.py:12`) has 9 fields; 6 are frontier-only
  (`min_frontier_size`, `min_frontier_dist`, `max_frontier_dist`,
  `frontier_buffer_cells`, `goal_inset_m`, `prefer_farthest`) vs 3 general
  (`preferred_goal_distance`, `blacklist_radius`, `max_explore_radius`). Worse, the
  NODE declares the frontier params as ROS params in `__init__`
  (`min_frontier_size`, `frontier_buffer_cells`, `max_frontier_dist`, …), so the
  manager hardcodes frontier tuning knowledge. Hello-world needs one param yet
  inherits all nine; the node declares six it never uses.

* **Tests already done.** F22 T01 critique (see `04-tasks/notdone/TF22-*.md`) read
  all four seam files and confirmed leaks (A), (B), (C). The hello-world plugin is
  the forcing function: its need to fake cluster state, wait out patience it does
  not need, and ignore six params is direct evidence.

* **Latest theory / direction.** One decoupling program, three coordinated changes:
  - **(A+B)** Replace the return with an intent-carrying result — enum or small
    dataclass `NEW_GOAL(xy) / NO_TARGETS_BLOCKED / EXPLORED_DONE`. The algorithm
    declares done; the node keeps only debounce + blacklist + timeout + stuck. This
    removes both the cluster-count peek and the frontier-shaped done-rule.
  - **(A)** Move visualization/telemetry off the protocol: an optional opaque
    `diagnostics()` (or a `viz` payload the node renders blindly) so a plugin with
    no clusters supplies nothing rather than faking `[]`. OPEN QUESTION: who owns
    frontier *marker* rendering? Drawing cluster-colored markers inherently needs to
    know they are clusters — likely the algorithm publishes its own markers, or
    supplies an opaque marker payload.
  - **(C)** Split params: a small general/session set stays shared
    (`max_explore_radius`, `blacklist_radius`, `preferred_goal_distance`); frontier
    tuning moves into a `FrontierParams` the `FrontierAlgorithm` owns. DESIGN
    CONSTRAINT: ROS params must be declared by the node to be settable from
    yaml/launch, so an algorithm's params need a declaration path — either the
    algorithm exposes a param schema the node declares generically, or the node
    hands the algorithm a handle to self-declare in its own namespace.

  Scope touches the protocol, `FrontierAlgorithm`, the node's done/marker/telemetry
  paths, `ExploreParams`, the node's ROS-param declarations, and
  `test_frontier_algorithm.py` + `test_explorer_manager_node.py`. Deliberately NOT
  folded into F22 (would balloon the hello-world scope); F22 proceeds on the current
  interface, and its fakery is the evidence. End state: `explorer_manager_node.py`
  imports `FrontierAlgorithm` only as the default in `main()`'s registry, with zero
  frontier concepts elsewhere.
