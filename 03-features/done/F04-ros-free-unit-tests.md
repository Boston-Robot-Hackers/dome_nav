# F04 — ROS-Free Unit Tests for Manager Logic

> **Partially reversed by F07 (2026-06-17).** The `SlamManager` extraction was undone:
> it wrapped ~6 lines of trivial state (a bool + one `makedirs`), so the wrapper +
> property-proxies + separate test file cost more than they saved. Its logic was folded
> back into `SlamManagerNode` (now a `LifecycleNode`) and `dome_nav/slam_manager.py` +
> `test/test_slam_manager_pure.py` were deleted. `NavManager` is **kept** — it holds real
> algorithms (nearest-target, localization score, intent parsing) with 21 pure tests that
> remain valuable. Node property-proxies were also removed; tests reach the manager directly.

**Priority**: Medium
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes (23 pure Python tests)
**Test Passing:** yes (48/48 total including existing ROS node tests)
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
