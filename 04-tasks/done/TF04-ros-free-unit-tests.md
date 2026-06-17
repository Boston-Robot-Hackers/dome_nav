# TF04 — ROS-Free Unit Tests (Feature F04)

## T01 — extract SlamManager pure Python class
**Status**: done
**Description**: Create `dome_nav/slam_manager.py` with `SlamManager` class containing
map_ready state, `on_map_received()` returning status string, `should_save()`, and
`ensure_map_dir()`. No rclpy imports.
**Test**: `test_slam_manager_pure.py` — 8 tests: init state, should_save before/after map,
on_map_received return value and idempotency, ensure_map_dir creates parent, no-parent case.

## T02 — extract NavManager pure Python class
**Status**: done
**Description**: Create `dome_nav/nav_manager.py` with `NavManager` class containing
`on_targets(json_str)`, `parse_intent(json_str)`, `find_nearest_confirmed(label, robot_xy)`,
and `navigate_status(label, target)`. No rclpy imports.
**Test**: `test_nav_manager_pure.py` — 15 tests: JSON parsing, invalid JSON, unknown action,
target filtering, distance calculation from origin and non-origin, navigate_status strings.

## T03 — refactor nodes as thin wrappers
**Status**: done
**Description**: Update `slam_manager_node.py` and `nav_manager_node.py` to delegate to
pure classes. Add property/setter pairs (`map_persist_path`, `confirmed_targets`) so
existing node tests can still set state directly. Nodes own ROS I/O only.
**Test**: existing `test_slam_manager.py` (7 tests) and `test_nav_manager.py` (18 tests)
must continue passing unchanged — they verify the node delegation is correct.

## T04 — verify all tests pass
**Status**: done
**Description**: Run all 48 tests (25 ROS node + 23 pure). Pure tests must pass with
no ROS environment (no rclpy init, no running nodes).
**Test**: `python3 -m pytest test_slam_manager_pure.py test_nav_manager_pure.py` — 23 pass
in 0.06s. Full suite 48/48.
