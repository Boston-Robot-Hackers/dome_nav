# TF01 — SLAM Bringup and Map Persistence (Feature F01)

## T01 — colcon build passes
**Status**: done
**Description**: `colcon build --packages-select dome_nav` completes without errors.
Fix any import errors, missing deps, or setup.py issues.
**Test**: build succeeds; `ros2 pkg list | grep dome_nav` returns the package.

## T02 — slam_toolbox launches via dome_nav
**Status**: done
**Description**: `bl dome_nav robot.launch.py` starts slam_toolbox. Verify
`/map` topic appears and `map→odom` TF edge is published.
**Test**: manual integration test in `test/test_map_validation.py` (marked `manual`, requires live stack) —
subscribes `/map`, asserts resolution > 0, width > 0, height > 0, at least some free (0) and occupied (100) cells exist.
Also asserts `map→odom` TF available via tf2. Test must pass before T02 is done.

## T03 — slam_manager_node publishes status
**Status**: done
**Description**: `/dome_nav/slam_status` publishes "mapping" once `/map` is received.
**Test**: `ros2 topic echo /dome_nav/slam_status` shows "mapping" after map arrives.

## T04 — pose graph saves on clean shutdown
**Status**: done
**Description**: Ctrl-C triggers `slam_manager_node.save_map()`. Files
`~/.dome/slam_map.posegraph` and `~/.dome/slam_map.data` exist after shutdown.
**Test**: manual — check files exist and are non-empty after clean shutdown.

## T05 — map loads on next run
**Status**: done
**Description**: Relaunch with existing `~/.dome/slam_map.posegraph`. Pre-built
map walls visible in RViz immediately without robot movement.
**Test**: manual — `/map` topic appears within 5s of launch with non-empty grid.

## T06 — write unit tests for slam_manager_node
**Status**: done
**Description**: Test `save_map()` service call logic with mocked ROS2 client.
Test `on_map()` sets `map_ready` and publishes status. Put in `test/test_slam_manager.py`.
