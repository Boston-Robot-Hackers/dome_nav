# F02 — Intent-Driven Navigation

Feature file name: `F02-intent-navigation.md`

**Priority**: High
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes (unit T01–T04; T05 manual pending live stack)
**Test Passing:** yes (unit); T05 not yet run
**Description**: nav_manager_node receives JSON intents on `/intent`, looks up the
target in `/targets/confirmed`, finds the nearest match by distance, and sends a
`NavigateToPose` goal to Nav2 in `map` frame. Publishes `/dome_nav/nav_status`.
Cancellation uses a tracked GoalHandle. Goal result (success/failure) updates status.

## How to Demo

**Setup**: full stack running (`dome_nav robot`), map built, at least one confirmed
target in `/targets/confirmed`.

**Steps**:
1. `ros2 topic echo /dome_nav/nav_status` in one terminal
2. Publish intent: `ros2 topic pub --once /intent std_msgs/String '{"data": "{\"action\": \"go_to_object\", \"label\": \"chair\"}"}'`
3. Verify status shows `navigating:chair`, robot moves toward target
4. Publish cancel: `ros2 topic pub --once /intent std_msgs/String '{"data": "{\"action\": \"cancel_navigation\"}"}'`
5. Verify status shows `cancelled`, robot stops

**Expected output**: Nav2 executes goal, status transitions navigating → done/failed/cancelled.
