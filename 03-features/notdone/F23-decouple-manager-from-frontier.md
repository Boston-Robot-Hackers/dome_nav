# F23 — Decouple the Explorer Manager from the Frontier Algorithm

**Priority**: Medium
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: Make `explorer_manager_node.py` independent of `FrontierAlgorithm`
and of frontier concepts generally. The node should own ROS, Nav2, and the
exploration *session*; each algorithm should own its own decision logic, its
done-condition, its tuning params, and its visualization. End state: the node
imports `FrontierAlgorithm` only as the default entry in `main()`'s selection
registry, with zero frontier-specific logic, fields, or params anywhere else.

Converted from issue I12 (see `05-issues/closed/I12-*`), which holds the full
critique that motivated this. Companion to F22 (hello-world plugin) — F22's need to
fake cluster state, wait out patience it doesn't need, and ignore six unused params
is the concrete evidence for each leak fixed here.

---

## Motivation

The F12 seam claims strategy-agnosticism but leaks frontier assumptions into the
manager on three fronts:

- **Output** — `next_goal -> (x,y) | None`; `None` conflates "blocked this tick"
  with "done," so the node peeks `algorithm.latest_clusters` to tell them apart, and
  reads frontier-specific `latest_clusters`/`latest_diag` in six places.
- **Done-decision** — the node hardcodes a frontier-shaped done-rule
  (`NO_FRONTIER_PATIENCE` empty ticks + `raw_clusters == 0`). "When am I done" is
  algorithm-specific. (Goal timeout and stuck detection are NOT — they react to
  navigation and correctly stay in the node.)
- **Input** — `ExploreParams` is 6/9 frontier-only, and the node declares those
  frontier params as ROS params itself.

---

## Scope

1. **Intent-carrying result.** Replace the `next_goal` return with a result that
   names the outcome (new goal / no targets right now / exploration complete) so the
   algorithm — not the node — declares done. Node keeps only mechanical session
   policy (debounce, blacklist, timeout, stuck).
2. **Visualization & diagnostics off the protocol.** Remove
   `latest_clusters`/`latest_diag` as required protocol surface; a plugin with no
   clusters supplies nothing. Decide and implement marker ownership (algorithm
   publishes its own, or supplies an opaque payload the node renders blindly).
3. **Split params.** A small shared/session param set stays general
   (`max_explore_radius`, `blacklist_radius`, `preferred_goal_distance`); frontier
   tuning moves into a `FrontierParams` the algorithm owns. Provide a
   ROS-param-declaration path for algorithm-owned params (params must be
   node-declared to be yaml/launch-settable).
4. **Verify decoupling.** No frontier concept (`frontier`, `cluster`, `Frontier*`)
   appears in `explorer_manager_node.py` outside `main()`'s registry.

**Out of scope:** new algorithms beyond the existing frontier + hello-world;
changing frontier detection math; the Explore-vs-Exploration naming rename (separate
chore).

---

## How to Demo

**Setup**: unit tests + a harness/robot run with both algorithms.

**Steps**:
1. `pytest` green: frontier + hello-world both satisfy the new result-typed protocol;
   no test references `latest_clusters` through the node.
2. `grep -iE 'frontier|cluster' dome_nav/explorer_manager_node.py` → only the
   `main()` registry line.
3. Launch `explore_algorithm:=hello`: declares done immediately after its one goal
   (no 14-tick wait); frontier launch behaves as before.

**Expected output**: The manager runs both algorithms with no frontier knowledge of
its own; each algorithm owns its done-condition, params, and visualization. All
tests pass; frontier behavior unchanged from the F10/F11 baseline.
