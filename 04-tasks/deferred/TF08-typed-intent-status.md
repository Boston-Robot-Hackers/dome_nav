# TF08 — Typed Intent/Status Messages for F08

Task file name: TF08-typed-intent-status.md
Cross-package change: dome_nav + dome_control + dome_vision must change together.

## T01 — Create dome_nav_msgs interface package
**Status**: not done
**Description**: Create `dome_nav_msgs/` sibling package with `Intent.msg` and `NavStatus.msg`.
Fields per F08 spec. Add `package.xml`, `CMakeLists.txt`. Build with colcon.
**Test**: `ros2 interface show dome_nav_msgs/msg/Intent` and `dome_nav_msgs/msg/NavStatus` both resolve.

## T02 — Update dome_nav to publish/subscribe typed messages
**Status**: not done
**Description**: In `nav_manager_node.py`, replace `std_msgs/String` subscriber on `/intent`
with `dome_nav_msgs/Intent`. Replace `std_msgs/String` publisher on `/dome_nav/nav_status`
with `dome_nav_msgs/NavStatus`. Remove `parse_intent()` JSON path and inline status
string formatting. Call `navigate_status()` only to derive `state` + `label` fields.
**Test**: Unit test that `NavStatus` publisher emits correct `state` and `label` fields
for idle, navigating, done, failed, cancelled, no_target cases.

## T03 — Update dome_nav to use typed /targets/confirmed
**Status**: not done
**Description**: Evaluate existing `vision_msgs` or common ROS2 types for
`/targets/confirmed`. If suitable type exists, adopt it. If not, add `Targets.msg`
to `dome_nav_msgs`. Update `nav_manager_node.py` subscriber accordingly.
Remove JSON parsing in `on_targets()`.
**Test**: Unit test that on_targets correctly populates confirmed targets from typed msg.

## T04 — Coordinate dome_control update (cross-package)
**Status**: not done
**Description**: Update dome_control to publish `dome_nav_msgs/Intent` on `/intent`
and subscribe `dome_nav_msgs/NavStatus` on `/dome_nav/nav_status`. Must be done
in lockstep with T02 — do not merge T02 without this.
**Test**: Integration smoke test: publish typed Intent, verify typed NavStatus received.

## T05 — Coordinate dome_vision update (cross-package)
**Status**: not done
**Description**: Update dome_vision `semantic_map_node.py` to publish typed message
on `/targets/confirmed` (matching whatever type was chosen in T03).
Must be done in lockstep with T03.
**Test**: Integration smoke test: dome_vision publishes typed targets, dome_nav receives.

## T06 — Delete dead String-based code paths
**Status**: not done
**Description**: After T02–T05 land: remove `parse_intent()`, remove colon-encoded
status formatting, remove any `json.loads` in nav_manager. Confirm no String producers
remain.
**Test**: `grep -r "json.loads\|parse_intent\|navigating:" dome_nav/` returns nothing.

## T07 — Update tests and literate docs
**Status**: not done
**Description**: Update `test_nav_manager_pure.py` and `test_nav_manager.py` to use
typed messages. Regenerate `01-literate/nav_manager_node.md`.
**Test**: All 55+ tests pass with `python3 -m pytest src/dome_nav/test/ -m "not manual"`.
