# Architecture & design critique: explorer_manager_node + algorithms

Date: 2026-07-16
Scope: `dome_nav/explorer_manager_node.py`, `dome_nav/frontier_algorithm.py`,
`dome_nav/hello_world_algorithm.py`, and supporting modules.

---

## Executive summary

The split is basically sound: a generic manager node owns navigation/session
lifecycle, while `FrontierAlgorithm` and `HelloWorldAlgorithm` plug in behind a
small protocol. The F23 decoupling moved most frontier tuning out of the node,
but the node still leaks frontier vocabulary and has a couple of real runtime
issues (goal-callback races, an undeclared ROS parameter). Overall it is a
workable plugin architecture with cleanup still to do.

---

## What is good

| Aspect | Why it works |
|--------|--------------|
| **Protocol seam** (`ExplorationAlgorithm` / `GoalDecision`) | `next_goal` returning `NEW_GOAL \| NO_TARGETS_BLOCKED \| EXPLORED_DONE` lets the algorithm own the done-condition without the node peeking at internals. |
| **Parameter split** | `ExploreParams` for shared/session tuning, `FrontierParams` for frontier-only tuning. The frontier algorithm declares its own ROS params, so yaml/launch overrides still work without the node knowing the names. |
| **Lazy CPU-sensitive resources** | TF listener and `OccupancyGrid` subscriptions are created/destroyed on demand. This is a real win on a Pi and in the 1-core dev VM. |
| **Costmap-bounds guard** | Re-asking the algorithm when a candidate goal maps outside the global costmap prevents the Nav2 `worldToMap` failure loop. |
| **Pure frontier functions** | `frontier_explorer.py` has no ROS dependency, so it is cheap to unit-test. |
| **Blacklist retry policy** | Clearing the blacklist once on a block before giving up is a pragmatic recovery. |

---

## Concrete bugs

### 1. Goal-result callback can corrupt the *next* active goal (race)

**File:** `dome_nav/explorer_manager_node.py`, `on_goal_result()` and `clear_active_goal()`.

When a goal is stuck or times out, `check_stuck()` / `check_goal_timeout()` cancel
the goal and clear the active-goal state. The next tick can then send a new goal.
The old goal's async result callback can still arrive later and unconditionally
calls `self.clear_active_goal()`, wiping `self.goal_handle`, `self.current_goal_xy`,
etc. — but those now belong to the *new* goal.

**Impact:** the node loses track of an active Nav2 goal and may dispatch a second
overlapping goal.

**Fix:** in `on_goal_result()`, verify the callback matches the current active goal
before clearing state:

```python
def on_goal_result(self, future, xy: XY):
    if xy != self.current_goal_xy or not self.has_active_goal:
        # Stale callback for a canceled/superseded goal.
        return
    ...
    self.clear_active_goal()
    ...
```

---

### 2. `blacklist_radius` is not declared as a ROS parameter

**File:** `dome_nav/explorer_manager_node.py` `__init__()` and
`dome_nav/explore_context.py` `ExploreParams`.

`ExploreParams.blacklist_radius` defaults to `0.5`, but the node only declares
`max_explore_radius`, `preferred_goal_distance`, and `map_name`. So
`blacklist_radius` cannot be changed from launch/yaml; it is silently pinned to
`0.5`.

**Fix:** declare it and pass it into `ExploreParams`:

```python
self.declare_parameter("blacklist_radius", 0.5)
self.params = ExploreParams(
    max_explore_radius=self.get_parameter("max_explore_radius").value,
    blacklist_radius=self.get_parameter("blacklist_radius").value,
    preferred_goal_distance=self.get_parameter("preferred_goal_distance").value,
)
```

---

### 3. User-initiated `exploration_stop` can still blacklist the canceled goal

**File:** `dome_nav/explorer_manager_node.py`, `stop_exploring()` / `on_goal_result()`.

`stop_exploring()` cancels the goal but does not prevent the pending result
callback from treating it as a failure and adding it to the blacklist. A
deliberate user stop should not mark the goal as failed or blacklist it.

**Fix:** same stale-callback guard as bug #1, or track a goal/session serial and
ignore callbacks from the previous session.

---

## Design issues & technical debt

### 4. The generic node still speaks "frontier"

**File:** `dome_nav/explorer_manager_node.py` (throughout).

Despite the decoupling intent, the manager is full of frontier-specific names:

- method `find_and_send_frontier()`
- method `dump_frontier_exhaustion()`
- counter `no_frontier_count`
- constant `NO_FRONTIER_PATIENCE`
- telemetry event key `"no_frontier"`
- default algorithm `FrontierAlgorithm()`

This makes the node less generic than the protocol suggests. `current.md` already
flags this as F23 T04; the rename should probably be `find_and_send_goal`,
`dump_exhaustion_report`, `no_goal_count`, `GOAL_PATIENCE`, `"no_goal"`.

---

### 5. Optional plugin hooks are invisible to type checkers

**File:** `dome_nav/explore_context.py` and `dome_nav/explorer_manager_node.py`.

`render_markers`, `exhaustion_report`, `failure_report`, `telemetry_extra`, and
`session_params` are documented in comments but not in the `ExplorationAlgorithm`
Protocol. The node calls them via `getattr(..., "render_markers", None)`. This
works, but:

- Renaming a hook silently breaks plugins.
- No mypy/autocomplete help for plugin authors.
- Magic strings are scattered in `algorithm_report`, `algorithm_telemetry`,
  `session_start_params`.

**Better:** either add the hooks to the Protocol with default no-op
implementations (e.g., via a mixin/ABC), or at least define string constants.

---

### 6. `MapInfo` lives in a frontier-specific module

**File:** `dome_nav/explore_context.py` imports `MapInfo` from
`dome_nav.frontier_explorer`.

The generic protocol depends on a type defined in the frontier implementation.
Moving `MapInfo` to `explore_context.py` (or a neutral `grid_types.py`) removes
that coupling.

---

### 7. Blacklist uses exact float tuples

**File:** `dome_nav/explorer_manager_node.py`, `self.blacklist`.

Goals are blacklisted as `(x, y)` float tuples. Rounding/floating-point noise
means two goals at the same cell can produce slightly different world
coordinates and not deduplicate. The *selection* side uses `blacklist_radius`,
but the *storage* side does not, so the set can accumulate near-duplicates.

**Better:** store discretized grid coordinates, or deduplicate on insert using the
existing radius.

---

### 8. Costmap guard only checks bounds, not cost

**File:** `dome_nav/explorer_manager_node.py`, `goal_in_global_costmap()`.

The guard returns `True` for any goal inside the costmap rectangle, even if the
cell is lethal/occupied. After `nudge_toward_robot()` pulls the goal inward, it
could land on an obstacle. Adding a lethal-cost check would catch that before
dispatch.

---

### 9. `NO_FRONTIER_PATIENCE` is duplicated

**File:** `dome_nav/explorer_manager_node.py` and `dome_nav/frontier_algorithm.py`.

`frontier_algorithm.py` hardcodes its own `NO_FRONTIER_PATIENCE = 14` for the
exhaustion report. If the node constant changes, the report becomes misleading.
It should come from the `RenderContext` or `session_params`.

---

### 10. No runtime algorithm selector

**File:** `dome_nav/explorer_manager_node.py` `__init__()`.

`HelloWorldAlgorithm` is fine as a reference plugin, but there is no way to
select it at launch; `ExplorerManagerNode()` hardcodes `FrontierAlgorithm()`.
`current.md` already lists F22 T03/T04 for this.

---

### 11. `NAV2_ERROR_CODES` is hand-maintained

**File:** `dome_nav/explore_diagnostics.py`.

The error-code table can drift with Nav2 versions. If `nav2_msgs` exposes named
constants, prefer those.

---

### 12. `wait_for_message` blocks the executor

**File:** `dome_nav/explorer_manager_node.py`, `fetch_grid()`.

On-demand grid fetching avoids idle CPU, but `wait_for_message` blocks the
single-threaded executor for up to 1 s. With `EXPLORE_HZ = 1`, a slow publisher
can stall the whole node. On latched topics this is normally fast, but it is
worth documenting/monitoring.

---

## Minor / nitpicks

- `TelemetryWriter` is created before algorithm params are declared; if param
  declaration throws, an empty telemetry file is left behind.
- `HelloWorldAlgorithm.next_goal()` uses `preferred_goal_distance` as a step size
  in map +x; that is a wiring demo, but the name is semantically odd — it has
  nothing to do with preferred distance to a frontier.
- `FrontierAlgorithm.render_markers()` uses `self.frontier_params.min_frontier_size`
  instead of the merged `tuning`, so a shared override cannot affect marker
  filtering. Probably fine, but inconsistent with `next_goal`.

---

## Recommended priority order

1. Fix the `on_goal_result` race (bug #1) — it can produce phantom goals.
2. Declare `blacklist_radius` (bug #2) — one-line fix, currently not tunable.
3. Ignore result callbacks after user stop (bug #3).
4. Finish F23 T04 rename to remove frontier vocabulary from the node.
5. Move `MapInfo` out of `frontier_explorer.py`.
6. Type the optional hooks in the Protocol or a mixin.
7. Make the algorithm selectable at runtime (F22 T03/T04).
8. Improve blacklist storage and the costmap guard.

---

## Update vs. pulled code (2026-07-16)

The latest source is mostly doc/naming cleanup. Below is a quick status check of
the original proposals against the new code.

| # | Proposal | Status | Notes |
|---|----------|--------|-------|
| 1 | Fix `on_goal_result` race | **FIXED** (2026-07-18) | `on_goal_result()` now early-returns on a stale callback (`not has_active_goal`, `xy != current_goal_xy`, or `goal_start_time is None`) before touching state. Fixed a live `float - None` crash; regression tests added. |
| 2 | Declare `blacklist_radius` | **NOT FIXED** | `ExploreParams.blacklist_radius` still defaults to `0.5`; the node only declares `max_explore_radius`, `preferred_goal_distance`, and `map_name`. Still not yaml-tunable. |
| 3 | Ignore result callbacks after user stop | **FIXED** (2026-07-18) | Same stale-callback guard as #1: after `stop_exploring()` clears goal state, the late `on_goal_result()` early-returns instead of blacklisting/counting the canceled goal. |
| 4 | Remove frontier vocabulary from node | **PARTIALLY FIXED** | Renamed: `NO_FRONTIER_PATIENCE → NO_TARGET_PATIENCE`, `no_frontier_count → no_target_count`, `find_and_send_frontier → find_and_send_goal`, `dump_frontier_exhaustion → dump_exhaustion`, `handle_no_frontier → handle_no_target`. Telemetry/status keys `"no_frontier"` / `"no_frontier_ticks"` are intentionally kept as wire contracts with migration comments. |
| 5 | Type optional hooks in Protocol | **NOT FIXED** | Hooks are still called via `getattr` with magic strings; `ExplorationAlgorithm` Protocol only lists `next_goal` and `declare_params`. |
| 6 | Move `MapInfo` out of `frontier_explorer.py` | **FIXED** | `MapInfo` now lives in `explore_context.py`. |
| 7 | Improve blacklist float-tuple storage | **NOT FIXED** | Still exact `(x, y)` float tuples; near-duplicate goals can accumulate. |
| 8 | Costmap guard should check cost, not just bounds | **NOT FIXED** | `goal_in_global_costmap()` still only checks whether the cell is inside the costmap rectangle. |
| 9 | Remove duplicated `NO_FRONTIER_PATIENCE` | **FIXED** | `RenderContext` now carries `patience`, and `FrontierAlgorithm.exhaustion_report()` reads `rc.patience`. |
| 10 | Runtime algorithm selector | **NOT FIXED** | `ExplorerManagerNode()` still hardcodes `FrontierAlgorithm()`; `HelloWorldAlgorithm` is not selectable at launch. |
| 11 | Replace hand-maintained Nav2 error codes | **NOT FIXED** | `NAV2_ERROR_CODES` table unchanged. |
| 12 | Document/monitor `wait_for_message` blocking | **NOT FIXED** | Behavior unchanged; still worth documenting. |

### New observations

- **Intentional wire-contract comments** are a good idea. Keeping `"no_frontier"`
  telemetry and `"no_frontier_ticks"` status keys for backward compatibility is
  defensible, but plan a migration (new keys + deprecation notice) so consumers
  can switch.
- **The remaining NOT FIXED items are the highest-risk ones.** The race in
  `on_goal_result` is the only item that can produce actively wrong robot
  behavior; the missing `blacklist_radius` declaration is the only one that
  silently ignores user tuning.
