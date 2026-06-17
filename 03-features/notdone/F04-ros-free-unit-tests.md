# F04 — ROS-Free Unit Tests for Manager Logic

**Priority**: Medium
**Done:** no
**Tasks File Created:** no
**Tests Written:** no
**Test Passing:** no
**Description**: Extract core logic from `slam_manager_node.py` and `nav_manager_node.py`
into pure Python classes with no rclpy dependency. ROS nodes become thin wrappers.
Pure Python classes are unit-testable without a ROS installation or running nodes.
Follows the dome_vision pattern (dome_vision = pure lib, dome_vision_ros = ROS wrapper).

## Scope

**New files:**
- `dome_nav/slam_manager.py` — pure Python: map-ready state, save/load path logic
- `dome_nav/nav_manager.py` — pure Python: intent parsing, status state machine,
  goal lifecycle (pending/active/cancelled/done/failed)

**Modified files:**
- `dome_nav/slam_manager_node.py` — thin ROS wrapper: spin, subscriptions, service
  calls delegate to `SlamManager`
- `dome_nav/nav_manager_node.py` — thin ROS wrapper: spin, action client, topic
  subscriptions delegate to `NavManager`

**New tests (no rclpy, no ROS running):**
- `test/test_slam_manager.py` — state transitions, path validation, save-skip-when-not-ready
- `test/test_nav_manager.py` — intent parsing (valid/invalid JSON), status transitions,
  cancel-while-navigating, no-target handling

## What does NOT change

- ROS interface (topics, actions, params) — identical
- launch files — no change
- Existing tests that do require ROS — kept as-is

## How to Demo

**Setup**: just Python — no ROS, no robot, no rosbag.

**Steps**:
1. `cd dome_nav && python -m pytest test/test_slam_manager.py test/test_nav_manager.py -v`
2. All tests pass with no ROS environment variables set

**Expected output**: full test suite passes in ~1 second. No rclpy imports anywhere
in `slam_manager.py` or `nav_manager.py`.

## Test plan

- `SlamManager`: init state=waiting, on_map_received → state=mapping, save skipped
  when not ready, save called when ready
- `NavManager`: parse `go_to_object` intent, `cancel_navigation` intent, malformed
  JSON, unknown action; status machine: idle→navigating→done/failed/cancelled
