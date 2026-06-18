# TF09 — dome_control ↔ dome_nav Integration for F09

## T01 — Fix parse_intent() to use "name" key
**Status**: done
**Description**: In `nav_manager.py:34`, change `intent.get("action", "")` to
`intent.get("name", "")`. Extract label from `intent.get("slots", {}).get("label", "")`
instead of `intent.get("label", "")` — dome_control puts label in slots.
Update `on_intent` in `nav_manager_node.py:63` to read label from slots accordingly.
**Test**: Update `test_parse_intent_go_to_object` and `test_parse_intent_cancel` to use
`{"name": ..., "slots": {...}}` format. Add test for missing slots key (should still parse).

## T02 — Close I02–I05 (already fixed in code)
**Status**: done
**Description**: Verify I02–I05 fixes are present in current source. Run tests to confirm
coverage. Move issue files from `05-issues/open/` to `05-issues/closed/`.
**Test**: `python3 -m pytest src/dome_nav/test/ -m "not manual"` passes.

## T03 — Add go_to_object + cancel_navigation commands to dome_control
**Status**: done
**Description**: In dome_control, add `go_to_object <label>` and `cancel_navigation`
as CLI commands that publish the correct intent. This is a dome_control change.
Confirm the published payload matches `{"name": "go_to_object", "source": "cli",
"slots": {"label": "<label>"}}`.
**Test**: Unit test that IntentPublisher.publish("go_to_object", slots={"label": "chair"})
produces correct JSON.

## T04 — Live integration smoke test
**Status**: not done
**Description**: With dome_nav + dome_control running (Mode B), issue
`go_to_object chair` from dome_control CLI. Verify dome_nav logs navigation attempt
and publishes nav_status. Mark manual.
**Test**: Manual — record command + observed log output + nav_status value.
