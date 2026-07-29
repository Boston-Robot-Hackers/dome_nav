# TF10 — Autonomous Exploration for F10

**Trimmed 2026-07-29**: T01–T05 + T08 are shipped; their descriptions are kept for
traceability but frame historical decisions (`explore_lite` dropped, node converged
onto the pluggable explorer). Only live-hardware verification (T06/T07) remains, and
it is blocked on the start-wedge problem (F29). T08's orphan-node removal is **done**
— the original file was removed and the pluggable node renamed to
`explorer_manager_node.py` in commit `206a93f` (verified 2026-07-29); the old
"pending removal" note was stale.

## T01 — Verify explore_lite available
**Status**: N/A — superseded
**Description**: Originally planned to use `ros-jazzy-explore-lite`. Decided instead to
build custom `frontier_explorer.py` (pure Python, no external ROS package dependency).
This task is moot.

## T02 — Add explore_param_patch.yaml
**Status**: done
**Description**: `config/explore_param_patch.yaml` created with conservative exploration
settings: `desired_linear_vel` 0.12 m/s, `max_velocity` [0.15, 0.0, 1.0], plus
frontier params (`MIN_FRONTIER_SIZE`, `MIN_FRONTIER_DIST`, `BLACKLIST_RADIUS`,
`GOAL_INSET_M`, `max_explore_radius`).

## T03 — Create robot_explore.launch.py
**Status**: done
**Description**: `launch/robot_explore.launch.py` (Mode E) exists. Requires `map_name`
arg (error if missing). Includes slam_toolbox online_async + nav2 + explore_manager_node.
Accepts `max_explore_radius` arg (default 0.0 = unlimited).

## T04 — Create explore_manager_node.py
**Status**: done
**Description**: `dome_nav/explore_manager_node.py` subscribes `/intent`, routes
`exploration_start` / `exploration_stop` intents → Nav2 NavigateToPose goals via
custom `frontier_explorer.py`. Blacklisting, nudge inset, 2 Hz timer loop,
publishes `/explore/status`. 84 tests pass including frontier_explorer pure tests.

## T05 — Add nav.explore / nav.explore.stop to dome_control
**Status**: done
**Description**: dome_control `nav explore` and `nav explore stop` publish
`exploration_start` / `exploration_stop` intents with correct JSON payload.

## T06 — Resolve open hardware questions
**Status**: not done — hardware, blocked on start-wedge (F29)
**Description**: Answer on hardware:
1. Does explore auto-stop cleanly when no frontiers remain?
2. What speed cap avoids slam_toolbox degradation? (real config now vx 0.4 m/s
   after the 2026-07-29 tuning — supersedes the original 0.12 m/s in T02.)
3. Do narrow doorways (<0.8 m) get traversed or blocked by costmap?
Record answers in `02-doc/notes.md`. Adjust params or auto-stop logic as needed.
**Test**: Manual — run Mode E on real robot, observe, record findings.

## T08 — Converge real robot onto pluggable_explore_manager_node
**Status**: done (launch switch) — orphaned original node not yet removed
**Description**: Per the standing 2026-07-04 architecture decision (sim and real should
share one explorer code path, differing only by parameter values), switched
`robot_explore.launch.py` from the original `explore_manager_node` to
`pluggable_explore_manager_node` (the same node the sim stack uses, with an injected
`FrontierAlgorithm`). Real-robot parameter values set explicitly in the launch:
`max_frontier_dist 0.0` (unlimited -- the node declares a sim-oriented 15.0, overridden
back here), `min_frontier_dist 1.3`, `prefer_farthest False`, `min_frontier_size 10`,
`max_explore_radius` from the launch arg, plus `use_sim_time` from the arg. These match
`ExploreParams`' own defaults, so behavior matches the original node's intent.
**Test**: `colcon build` clean; full suite 181 passed, 4 deselected. Not yet live on
hardware (real-robot run is T07).
**Resolved 2026-07-29**: the orphan removal is done. Commit `206a93f` renamed
`pluggable_explore_manager_node` → `explorer_manager_node.py` and the original
`explore_manager_node.py` no longer exists in the tree. `setup.py` points the
console entry at `explorer_manager_node`. Nothing left to delete.

## T07 — Manual live smoke test
**Status**: not done — hardware, blocked on start-wedge (F29)
**Description**: Full end-to-end on real robot:
1. `bl dome_nav robot_explore.launch.py --map_name basement_explore`
2. `nav explore` from dome_control CLI
3. Observe robot driving autonomously, map growing in Foxglove
4. `nav explore stop` → robot stops cleanly, map saved to `~/.dome/slam_maps/`
Record: `/explore/status` transitions seen (idle → exploring → done/idle), map file written.
**Test**: Manual — mark done only when all four observations confirmed.
