# F08 — Typed Intent / Status Messages (proposal)

**Priority**: Medium
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: Replace the `std_msgs/String` + embedded-JSON interfaces with a minimal
set of typed ROS messages. Today `/intent`, `/targets/confirmed`, and
`/dome_nav/nav_status` are all `String` carrying ad-hoc text/JSON. This discards ROS2's
type system, `ros2 topic echo` legibility, and IDL validation — and already caused a
spec/impl drift (spec says status `"done"`, code emits `"done:<label>"`).

## Why this is a proposal, not done

`/intent` and `/targets/confirmed` are **produced by dome_control and dome_vision**.
Changing dome_nav's message types breaks those packages unless all three change in
lockstep. This is a cross-package contract change and must be coordinated; it is not a
dome_nav-only edit.

## Minimal type set (avoid a message zoo)

Keep specialized types to the minimum:

- `dome_nav_msgs/Intent.msg`
  ```
  string action      # "go_to_object" | "cancel_navigation"
  string label       # object label; empty for cancel
  ```
- `dome_nav_msgs/NavStatus.msg`
  ```
  string state       # "idle" | "navigating" | "done" | "failed" | "cancelled" | "no_target"
  string label       # target label this state refers to; empty when not applicable
  ```
- `/targets/confirmed`: prefer an existing common type if one fits (e.g. a
  `vision_msgs` detection array) before minting a dome-specific one. Only add a custom
  type if nothing standard carries label + `xyz_world`.

`NavigateToPose` (Nav2 action) and the localization topics are already typed — no change.

## Migration

1. Create `dome_nav_msgs` interface package with `Intent.msg` + `NavStatus.msg`.
2. dome_nav publishes/subscribes the typed messages; drop the colon-encoded status and
   JSON parsing in `nav_manager.py`/`nav_manager_node.py`.
3. Update dome_control (`/intent` producer, `/dome_nav/nav_status` consumer) and
   dome_vision (`/targets/confirmed` producer) together.
4. Delete `parse_intent` JSON path and the `"navigating:<label>"` string formatting once
   no String producers remain.

## How to Demo

**Setup**: all three packages built against `dome_nav_msgs`.

**Steps**:
1. `ros2 interface show dome_nav_msgs/msg/Intent`
2. `ros2 topic echo /dome_nav/nav_status` — structured fields, not colon strings
3. Publish a typed Intent, verify navigation + typed status transitions

**Expected output**: introspectable typed topics; no JSON-in-String; spec and impl
cannot drift on status values because they are message fields.
