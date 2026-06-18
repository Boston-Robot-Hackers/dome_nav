# F03 — Static Map + AMCL Navigation Mode

**Priority**: High
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: Add Mode B to dome_nav: load a saved occupancy map via map_server,
localize with AMCL (particle filter, no initial pose needed), run Nav2 for path
planning. Replaces slam_toolbox as the runtime localization source. Robot can start
anywhere in the house and AMCL converges from lidar alone.

## Scope

- New `robot_nav.launch.py`: map_server + amcl + Nav2 navigation_launch (no slam_toolbox)
- New `nav2_amcl_patch.yaml`: AMCL params tuned for dome robot
- `nav_manager_node.py` verified to work with AMCL TF (map→odom source changes but
  interface is identical)
- Remove `map_start_at_dock` param from all configs
- Existing `robot.launch.py` renamed to `robot_map.launch.py` (Mode A, map build)

## What does NOT change

- `/dome_nav/nav_status` topic and semantics
- `/intent` subscription and go_to_object handling
- NavigateToPose action interface
- dome_control and dome_vision interfaces

## How to Demo

**Setup**: saved map exists at `~/.dome/slam_map.yaml` + `~/.dome/slam_map.pgm`
(from a prior Mode A run). linorobot2 bringup running (odom, lidar, TF tree).

**Steps**:
1. `colcon build --packages-select dome_nav && source install/setup.bash`
2. `bl dome_nav robot_nav.launch.py`
3. Open RViz/Foxglove — saved map should appear immediately
4. Watch AMCL particle cloud converge within ~30 seconds of lidar data
5. `ros2 topic echo /dome_nav/nav_status` — should show `"idle"`
6. Send a nav goal: `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...`
7. Verify robot plans path, moves, status transitions to `navigating` → `done`

**Expected output**: AMCL converges without fixed start pose. Nav2 executes goal.
No TF errors. No slam_toolbox running.

## Test plan (no dome_vision / dome_control needed)

- Unit: `nav_manager.py` pure Python logic — mock ROS calls, test status transitions
- Integration: play `/scan` + `/odom` rosbag, verify AMCL publishes `map→odom` TF
- Integration: with AMCL running, send `NavigateToPose` via CLI, verify status
