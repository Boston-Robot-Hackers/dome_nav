# F10 — Autonomous Exploration for Map Building

**Priority**: Medium
**Done:** no (implementation shipped + unit-tested; open only on real-robot
live verification — T06/T07 — which is blocked on the start-wedge problem, F29)
**Tasks File Created:** yes
**Tests Written:** yes (pure/unit; frontier_explorer + explorer node)
**Test Passing:** yes (full suite green)
**Description**: Allow the robot to autonomously explore an unknown space —
navigating slowly, visiting reachable areas, avoiding obstacles — so a map is built
without manual teleoperation. Frontier-based exploration integrated with the Mode A
(slam_toolbox online_async) stack.

**Trimmed 2026-07-29 to match reality.** The original plan used the `explore_lite`
ROS package; that was dropped (see TF10 T01). Exploration is a custom pure-Python
`frontier_explorer.py` driven by `explorer_manager_node.py` — the single node shared
by sim and real (F23 decoupling), differing only by ROS params. (This node is the
former `pluggable_explore_manager_node`, renamed in commit `206a93f`; the original
`explore_manager_node.py` was removed in that same convergence.) F10 is the
foundation later features build on: F15 novelty, F27 lethal-goal guard, F31
scoring pipeline are all tenants of this explorer.

## Scope (as shipped)

- `launch/robot_explore.launch.py` — Mode A stack (slam_toolbox online_async + Nav2)
  + `explorer_manager_node` with an injected `FrontierAlgorithm`; requires
  `map_name`; real-robot param values set explicitly in the launch.
- `dome_nav/explorer_manager_node.py` — subscribes `/intent`, routes
  `exploration_start` / `exploration_stop` → Nav2 `NavigateToPose` goals via
  `frontier_explorer.py`; blacklist, nudge inset, timer loop; publishes
  `/explore/status` (idle | exploring | done).
- `dome_nav/frontier_explorer.py` — pure frontier detection + F31 goal-scoring
  pipeline (filters + weighted scorers).
- dome_control `nav explore` / `nav explore stop` CLI → matching intent payloads.
- Exploration speed capped for slam stability (real config tuning, see current.md).

## Constraints

- Runs on top of Mode A (slam_toolbox online_async); not Mode B (AMCL only).
- Reuses the Nav2 costmap from the explore config.
- No dome_vision or dome_control node required to run the explorer.

## How to Demo

**Setup**: fresh `map_name`.

**Steps**:
1. `bl dome_nav robot_explore.launch.py --map_name newroom`
2. From dome_control CLI: `nav explore`
3. Watch robot drive autonomously, map grows in RViz/Foxglove
4. `nav explore stop`, or robot stops automatically when no frontiers remain
5. Map saved under `~/.dome/slam_maps/`

**Expected output**: occupancy grid of the space built with no manual driving.

## Remaining before close

Implementation and unit tests are done. What is left is **real-robot live
verification** (TF10 T06 hardware questions + T07 live smoke). Both are blocked on
the standing start-wedge problem — the robot stalls when it starts near an obstacle
(current.md; F29 BackUp escape targets the cure). F10 closes once the robot explores
hardware clean; the wedge is the gating dependency, not missing F10 code.

## Cleanup — done, not pending

Earlier docs claimed an orphaned `explore_manager_node.py` still needed removal.
Verified 2026-07-29: that file no longer exists. Commit `206a93f` renamed
`pluggable_explore_manager_node` → `explorer_manager_node.py` and the original
orphan was already gone. No cleanup outstanding.
