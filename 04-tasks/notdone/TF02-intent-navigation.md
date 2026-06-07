# TF02 — Intent-Driven Navigation (Feature F02)

## T01 — fix find_nearest_confirmed to return nearest by distance
**Status**: done
**Description**: Current implementation returns `matches[0]` (first match, not nearest).
Compute Euclidean distance from robot's current pose to each target's `xyz_world`,
return the closest. Requires subscribing to `/odom` or using TF to get robot pose in map frame.
**Test**: unit test — mock confirmed_targets list with two targets at different distances,
assert correct one returned.

## T02 — fix cancel_navigation to use tracked GoalHandle
**Status**: done
**Description**: Current `cancel_navigation` calls `self.nav_client._cancel_goal_async()`
which is a private rclpy API. Store the `GoalHandle` returned by `send_goal_async` and
call `goal_handle.cancel_goal_async()` instead.
**Test**: unit test — mock ActionClient, assert `goal_handle.cancel_goal_async()` called
on cancel intent.

## T03 — track goal result and publish done/failed status
**Status**: done
**Description**: `send_goal_async` result is not checked. Add a result callback that
publishes `done:<label>` on success or `failed:<label>` on failure/abort.
**Test**: unit test — mock goal result with SUCCESS and ABORTED, assert correct status
published.

## T04 — unit tests for nav_manager_node
**Status**: done
**Description**: Test `on_intent` routing, `navigate_to_object` (target found / not found),
`cancel_navigation`, `publish_status`. Mock ActionClient and confirmed_targets.
Put in `test/test_nav_manager.py`.
**Test**: plain pytest, no live stack required.

## T05 — manual integration test
**Status**: not done
**Description**: With live stack and a confirmed target, publish `go_to_object` intent.
Verify robot moves, status publishes correctly, cancel stops robot.
**Test**: manual — observe robot motion and `ros2 topic echo /dome_nav/nav_status`.
Cannot be automated without full dome_vision stack.
