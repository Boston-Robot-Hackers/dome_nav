# TF12 — Pluggable Exploration Algorithm for F12

Note: All new files are additive. The existing explore_manager_node.py and its tests
are left unchanged so the old non-pluggable approach keeps working side by side.

## T01 — Create explore_context.py
**Status**: done
**Description**: New file `dome_nav/dome_nav/explore_context.py`. Define `ExploreParams`
dataclass (min_frontier_size, blacklist_radius, min_frontier_dist, goal_inset_m,
max_explore_radius), `ExplorationContext` dataclass (map_data, map_info, robot_xy,
blacklist, start_xy, params), and `ExplorationAlgorithm` Protocol with `next_goal` method.
Import `MapInfo` from `frontier_explorer`. File header, MIT license, shebang per style guide.
**Test**: Import the module in a plain Python REPL — no ROS2 required. Verified.

## T02 — Create frontier_algorithm.py
**Status**: done
**Description**: New file `dome_nav/dome_nav/frontier_algorithm.py`. Implement
`FrontierAlgorithm` class with `latest_clusters` and `latest_diag` attributes and a
`next_goal(ctx)` method that calls `find_frontier_clusters`, `pick_best_frontier`,
`nudge_toward_robot`, and `_frontier_diag` from `frontier_explorer.py`. Sets
`latest_diag` when no frontier found, clears it otherwise.
**Test**: Covered by T04 (test_frontier_algorithm.py).

## T03 — Create pluggable_explore_manager_node.py
**Status**: done
**Description**: New file `dome_nav/dome_nav/pluggable_explore_manager_node.py`. Copy
from `explore_manager_node.py` and modify: accept `algorithm: ExplorationAlgorithm | None`
in `__init__`. Build `self.params = ExploreParams(...)` from ROS parameters. Default to
`self.algorithm = algorithm or FrontierAlgorithm()`. Replace direct calls to
`find_frontier_clusters`, `pick_best_frontier`, `nudge_toward_robot`, `_frontier_diag`
in `find_and_send_frontier` with `ExplorationContext` construction + `self.algorithm.next_goal(ctx)`.
Replace `self.latest_clusters` with `self.algorithm.latest_clusters` in `publish_markers`.
Remove class-level constants (MIN_FRONTIER_SIZE etc.) and now-unused frontier function imports.
Original `explore_manager_node.py` stays unchanged.
**Test**: Covered by T05.

## T04 — Create test_frontier_algorithm.py
**Status**: done
**Description**: New file `test/test_frontier_algorithm.py`. Pure Python,
zero ROS2. Reuse `make_info` / `flat_map` helpers from `test_frontier_explorer.py`.
Tests: `next_goal` returns None on fully-explored map; returns valid `(x, y)` on map
with frontier cells; `latest_clusters` populated after each call; `latest_diag` set
when returning None and cleared otherwise; blacklist respected; `goal_inset_m` nudge applied.
**Test**: `python3 -m pytest test/test_frontier_algorithm.py -v`

## T05 — Create test_pluggable_explore_manager_node.py
**Status**: done
**Description**: New file `test/test_pluggable_explore_manager_node.py`. Copy from
`test_explore_manager_node.py` and adapt: inject a `MockAlgorithm` at node construction
instead of patching module-level functions. Remove `PATCH_FFC` / `PATCH_PBF` patch
constants and `patch()` context managers. All assertions about state transitions,
telemetry, blacklisting, goal timeout, and marker publishing carried over. Original
`test_explore_manager_node.py` stays unchanged.
**Test**: `python3 -m pytest test/test_pluggable_explore_manager_node.py -m "not manual" -v`

## T07 — CLI demo: algo_demo.py
**Status**: done
**Description**: New file `tools/algo_demo.py`. Pure Python, no ROS2. Provides an
interactive terminal visualization of the FrontierAlgorithm running on small hand-crafted
maps. Features:
- ASCII art map rendering: `.` free, `#` occupied, `?` unknown, `F` frontier cell,
  `R` robot, `G` nudged goal, `B` blacklisted point
- Several built-in named maps (e.g. `room`, `corridor`, `ring`) selectable via CLI arg
- Step mode: press Enter to advance one tick; robot teleports to goal after each step
- After each step: prints cluster count, goal xy, diag dict if no frontier found
- Stops when algorithm returns None for NO_FRONTIER_PATIENCE consecutive ticks or map
  is fully explored
- Usage: `python3 tools/algo_demo.py [--map room|corridor|ring] [--inset 0.3]
  [--min-size 1] [--min-dist 0.0]`
**Test**: `python3 tools/algo_demo.py --map room` runs without error, shows ASCII map,
steps through until done.

## T06 — Full suite regression
**Status**: done
**Description**: Run complete pure-Python test suite. ROS2-dependent tests
(test_explore_manager_node.py, test_pluggable_explore_manager_node.py) require a ROS2
environment and must be run on the robot. Pure Python suite: 42 tests passing
(test_frontier_explorer.py + test_frontier_algorithm.py).
**Test**: `python3 -m pytest test/test_frontier_algorithm.py test/test_frontier_explorer.py -v`
