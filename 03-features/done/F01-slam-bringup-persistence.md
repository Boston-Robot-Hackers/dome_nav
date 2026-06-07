# F01 — SLAM Bringup and Map Persistence

**Priority**: High
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Launch slam_toolbox via dome_nav, verify map builds from lidar,
verify pose graph saves on shutdown, verify map loads on next run. This is the
foundation everything else depends on.

## How to Demo

**Setup**: linorobot2 bringup running (odom, lidar, TF tree). Robot on USB 3.

**Steps**:
1. `colcon build --packages-select dome_nav && source install/setup.bash`
2. `bl dome_nav robot.launch.py`
3. Drive robot around room — verify `/map` topic appears in RViz/Foxglove
4. `ros2 topic echo /dome_nav/slam_status` — should show "mapping"
5. Kill launch cleanly (Ctrl-C)
6. Check `~/.dome/slam_map.posegraph` exists
7. Relaunch — verify map loads (pre-built walls visible immediately)

**Expected output**: Map persists across restarts. slam_status publishes. No TF errors.
