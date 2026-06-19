# F10 — Autonomous Exploration for Map Building

**Priority**: Medium
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: Allow the robot to autonomously explore an unknown space — navigating slowly,
visiting all reachable areas, avoiding obstacles — so a complete map can be built without
manual teleoperation. Uses frontier-based exploration via `explore_lite` (or equivalent)
integrated with the existing Mode A (slam_toolbox online_async) stack.

## Scope

- `launch/robot_explore.launch.py` — Mode A stack + `explore_lite` node; accepts `map_name` arg (required)
- `config/explore_param_patch.yaml` — tuned exploration params: slow speed, conservative costmap inflation
- `dome_nav/explore_manager_node.py` — ROS2 node that:
  - subscribes `/intent` and recognizes `exploration_start` / `exploration_stop` intents
  - starts/stops `explore_lite` accordingly (via `/explore/start` and `/explore/stop`)
  - publishes `/explore/status` (idle | exploring | done)
- dome_control gets `nav.explore` and `nav.explore.stop` CLI commands publishing the matching intent payloads
- Integration with `nav_intent_check.py` or a new `explore_intent_check.py` tool for smoke-testing

## Constraints

- Must run on top of Mode A (slam_toolbox online_async); not compatible with Mode B (AMCL localization only)
- Robot must start at dock (same physical origin assumption holds)
- `explore_lite` requires Nav2 costmap to be running — reuse nav2_param_patch from Mode A
- Exploration speed capped: reduce `max_vel_x` in nav2 param patch for explore launches
- No dome_vision or dome_control required

## Dependencies

- `explore_lite` ROS2 package (`ros-jazzy-explore-lite` or built from source)
- Existing Mode A launch infrastructure (`robot_map.launch.py` pattern)

## How to Demo

**Setup**: empty `~/.dome/slam_maps/newroom/` (or just a fresh map_name).

**Steps**:
1. `bl robot_explore.launch.py --map_name newroom`
2. From dome_control CLI: `nav explore`
3. Watch robot drive autonomously, map grows in RViz/Foxglove
4. When coverage complete: `nav explore stop` or robot stops automatically (no more frontiers)
5. Map saved to `~/.dome/slam_maps/newroom`

**Expected output**: complete occupancy grid of the space with no manual driving required.

## Open questions

- Does `explore_lite` handle multi-room spaces with narrow doorways on linorobot2 hardware?
- Should exploration stop automatically (no frontiers) or require manual `stop` command?
- Speed cap value: need to tune on hardware to avoid slam_toolbox scan-matching failures at speed.
