# F22 — Hello-World Minimal Explorer Plugin

**Priority**: Low
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: A minimal reference exploration algorithm that plugs into the
F12 `ExplorationAlgorithm` protocol, plus a runtime mechanism to select it. Serves
as the copy-paste template for authoring new plugins and as an end-to-end proof
the injection seam actually swaps behavior on a real robot — not just in tests.

---

## Motivation

F12 built the pluggable seam (`ExplorationContext` in, `(x,y) | None` out) but:
- The only implementation is `FrontierAlgorithm` — full frontier clustering. Too
  much code to read as a "how do I write a plugin" example.
- `main()` (`explorer_manager_node.py:598`) always constructs the default
  `FrontierAlgorithm`. Injection exists only via the `__init__(algorithm=...)`
  constructor arg — reachable from unit tests, **not** from launch/runtime. So no
  one can actually run an alternate algorithm on the robot without editing code.

F22 closes both gaps: a trivial algorithm + a param to pick it.

---

## Scope

1. **Preliminary critique (do first).** Audit the F12 seam using the hello-world
   plugin as the forcing function — where does logic sit on the wrong side of the
   manager/algorithm boundary, is the interface complete, are names parallel? The
   critique may revise items 2–3 before any code. (Task-level checklist in TF22 T01.)
2. **`HelloWorldAlgorithm`** — a minimal reference plugin: ignores the map, emits
   one fixed step goal ahead of the robot, then declares done. The smallest thing
   that still drives the robot; heavily commented as the copy-paste template.
3. **Runtime selection** — an `explore_algorithm` ROS param + registry so the
   algorithm can be picked at launch, not only via the constructor. Default
   unchanged (`frontier`); production launches untouched.

**Out of scope:** new map/scan inputs or cmd_vel-direct navigation (F12 future
slots — separate features); any change to `FrontierAlgorithm` or the node's
session logic (blacklist, stuck/timeout, markers).

---

## How to Demo

**Setup**: driver stack + `bl dome_nav robot_explore.launch.py` (or the nav
harness) with `explore_algorithm:=hello`.

**Steps**:
1. `pytest test/test_hello_world_algorithm.py` — passes without ROS: first call
   returns a goal ~`preferred_goal_distance` ahead in +x; second call returns
   `None`.
2. Launch with `explore_algorithm:=hello`; start exploration.
3. Watch telemetry/logs: node sends exactly one nav goal a fixed step ahead, robot
   drives to it, then exploration declares done (no_frontier patience → done).
4. Relaunch with default (no param): frontier exploration behaves as before.

**Expected output**: Selecting `hello` at launch swaps the algorithm with no code
edit; the robot executes a single hard-coded step goal. Default launch is
byte-for-byte the old frontier behavior. All unit tests pass.
