# F09 — dome_control ↔ dome_nav Integration

**Priority**: High
**Done:** no
**Tasks File Created:** yes
**Tests Written:** no
**Test Passing:** no
**Description**: Wire dome_control and dome_nav so that navigation intents published by
dome_control are correctly received and acted on by dome_nav. Currently the intent key
mismatch (`"name"` vs `"action"`) means every dome_control intent is silently ignored
by dome_nav. Also close verified-fixed issues I02–I05.

## Scope

- Fix `parse_intent()` in `nav_manager.py` to read `"name"` key (dome_control contract)
  and accept `"slots"` dict for label extraction
- Update all tests that use the old `"action"` key format
- Add `navigation_go` and `navigation_cancel` as commands in dome_control that publish
  the correct intent payload
- Verify dome_control → `/intent` → dome_nav pipeline end-to-end on live robot
- Close I02–I05 (already fixed in code, just need issue files moved)

## Constraints

- dome_control intent JSON format is canonical: `{"name": ..., "source": ..., "slots": {...}}`
- Do NOT change dome_control's intent format — dome_nav must adapt to it
- dome_nav changes only; dome_control gets new commands added (not format changes)

## How to Demo

**Steps**:
1. Run dome_nav + dome_control together
2. Issue `navigation_go chair` from dome_control CLI
3. Verify dome_nav logs "Navigating to chair" and publishes nav_status

**Expected output**: intent flows from dome_control CLI → `/intent` → dome_nav →
NavigateToPose goal sent to Nav2.
