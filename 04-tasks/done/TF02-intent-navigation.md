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

## T06 — fix non-list JSON crash in on_targets (I02)
**Status**: done — isinstance(result, list) guard in nav_manager.py; regression tests test_on_targets_dict_json_rejected + test_on_targets_scalar_json_rejected pass
**Description**: on_targets() assigns json.loads() result without isinstance check. Non-list JSON (dict, scalar) sets confirmed_targets to wrong type; find_nearest_confirmed then raises AttributeError. Add isinstance(result, list) guard; log warning and return False if not list.
**Test**: add regression test to test_nav_manager_pure.py: on_targets with dict JSON, on_targets with scalar JSON.

## T07 — fix non-dict JSON crash in parse_intent (I03)
**Status**: done — isinstance(intent, dict) guard in nav_manager.py; regression tests test_parse_intent_list_json_rejected + test_parse_intent_string_json_rejected pass
**Description**: parse_intent calls intent.get() without isinstance check. Valid non-dict JSON (list, string) raises AttributeError. Add isinstance(intent, dict) check after json.loads; return None if not dict.
**Test**: add regression test to test_nav_manager_pure.py: parse_intent with list JSON, parse_intent with string JSON.

## T08 — fix missing xyz_world silent fallback (I04)
**Status**: done — nav_manager_node.py navigate_to_object checks xyz is None, logs warning, publishes no_target:label
**Description**: navigate_to_object falls back to [0,0,0] when xyz_world key absent, silently navigating to map origin. Check key exists first; log warning and publish no_target:label if missing.
**Test**: add regression test to test_nav_manager_pure.py: navigate_status when target lacks xyz_world.

## T09 — add warning log for malformed/unknown intent (I05)
**Status**: done — on_intent logs warning with raw msg.data when parse_intent returns None
**Description**: on_intent does bare return when parse_intent returns None, with no log. Add get_logger().warning with the raw message data so operators see diagnostic output.
**Test**: manual — send malformed JSON on /intent, verify warning appears in node log.

## T10 — remove leading underscore prefixes (I06)
**Status**: done — renamed _cov→make_cov (test_nav_manager_pure.py) and _cb→on_map (test_map_validation.py); source files had no violations; 26 tests pass
**Description**: nav_manager_node.py, slam_manager_node.py, utils.py all use leading underscore on methods and instance vars, violating MUST rule in style_guide.md. Rename all: _manager→manager, _goal_handle→goal_handle, _on_goal_accepted→on_goal_accepted, etc.
**Test**: all existing tests pass after rename.

## T05 — manual integration test
**Status**: done — test_nav_intent.py: 3/3 pass on live robot. Discovered and fixed AMCL QoS
mismatch (VOLATILE→TRANSIENT_LOCAL) in both nav_manager_node.py and test node.
