# TF10 — Autonomous Exploration for F10

## T01 — Verify explore_lite available
**Status**: not done
**Description**: Confirm `ros-jazzy-explore-lite` installs and its topics/services match
what we expect. Check: `/explore/start` and `/explore/stop` service names, costmap
topic it subscribes to, and whether it auto-stops when no frontiers remain.
Run `ros2 pkg list | grep explore` after install. Document actual topic/service names
in `02-doc/notes.md` — F10 scope assumes names that may differ.
**Test**: Manual — `apt install ros-jazzy-explore-lite`, verify pkg found, list its nodes and topics.

## T02 — Add explore_param_patch.yaml
**Status**: not done
**Description**: Create `config/explore_param_patch.yaml` with conservative exploration
settings: reduced `max_vel_x` (≤0.15 m/s), increased costmap inflation radius, and
any `explore_lite`-specific params (min_frontier_size, planner_frequency). Speed cap
is critical — slam_toolbox scan-matching degrades at high speed.
**Test**: Param file loads without error in launch (verified in T03).

## T03 — Create robot_explore.launch.py
**Status**: not done
**Description**: New launch file `launch/robot_explore.launch.py`. Pattern mirrors
`robot_map.launch.py`: requires `map_name` arg (error if missing, same error format).
Includes slam_toolbox online_async + nav2 (with explore_param_patch applied on top of
nav2_param_patch) + `explore_manager_node` + `explore_lite` node.
`explore_lite` starts paused — `explore_manager_node` triggers it via intent.
**Test**: `bl robot_explore.launch.py` (no map_name) → clear error message. Launch with
`--map_name test` on live stack → no crash, all nodes appear in `ros2 node list`.

## T04 — Create explore_manager_node.py
**Status**: not done
**Description**: ROS2 node `dome_nav/explore_manager_node.py`. Subscribes `/intent`,
acts on `exploration_start` (calls `/explore/start` service) and `exploration_stop`
(calls `/explore/stop` service). Publishes `/explore/status` (String: idle | exploring | done).
Transitions to `done` when explore_lite signals no more frontiers (subscribe to explore_lite
status topic — confirm name in T01). Lifecycle: clean shutdown on node destroy.
**Test**: Unit tests (mocked ROS2): intent routing, status transitions, service calls mocked.
Mark hardware tests manual.

## T05 — Add nav.explore / nav.explore.stop to dome_control
**Status**: not done
**Description**: In dome_control `navigation_commands.py`, add two commands:
- `nav.explore` → `publish_intent_exploration_start` → intent `{"name": "exploration_start", "source": "cli", "slots": {}}`
- `nav.explore.stop` → `publish_intent_exploration_stop` → intent `{"name": "exploration_stop", "source": "cli", "slots": {}}`
Add matching methods to `robot_controller.py`.
**Test**: Unit test in `test_command_dispatcher_text.py`: `nav explore` dispatches
`publish_intent_exploration_start`; `nav explore stop` dispatches `publish_intent_exploration_stop`.

## T06 — Resolve open questions from F10
**Status**: not done
**Description**: Answer on hardware before writing explore_manager auto-stop logic:
1. Does explore_lite auto-stop when no frontiers? What signal does it emit?
2. What speed cap avoids slam_toolbox degradation on linorobot2?
3. Do narrow doorways (<0.8m) get traversed or blocked?
Record answers in `02-doc/notes.md`. Update T04 implementation if auto-stop signal differs
from assumed.
**Test**: Manual — run T03 launch on real robot, observe behavior, record findings.

## T07 — Manual live smoke test
**Status**: not done
**Description**: Full end-to-end on real robot:
1. `bl robot_explore.launch.py --map_name basement_explore`
2. `nav explore` from dome_control CLI
3. Observe robot driving autonomously, map growing in Foxglove
4. `nav explore stop` → robot stops, map saved
Record: explore_lite node started, `/explore/status` transitions seen, map file written.
**Test**: Manual — mark done only when all four observations confirmed.
