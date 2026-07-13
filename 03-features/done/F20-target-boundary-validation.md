# F20 — Validate Nav Targets Once at the Boundary

**Priority**: Low
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Remove the "validate at every level + silent default" anti-pattern
for nav targets (review findings 1-2, 2026-07-13). Malformed targets are now
dropped once, at ingest (`on_targets`), and trusted everywhere downstream.

## Scope

1. `nav_manager.py` — `on_targets()` filters the parsed list to valid targets
   (dict with a numeric `xyz_world` of length >= 2) and stores only those.
   `find_nearest_confirmed()` drops its `target.get("xyz_world", [0,0,0])` silent
   default and reads `target["xyz_world"]` directly (trusted).
2. `nav_manager_node.py` — `navigate_to_object()` drops the post-selection
   `if xyz is None` re-check and the `float(...)` coercions; reads `xyz[0]`,
   `xyz[1]` directly. `yaw_world` stays an optional field (`.get(..., 0.0)`) but
   without `float()`.

## Constraints

- `/targets/confirmed` is external (from dome_vision) — validate-and-reject at the
  boundary is correct; reject (drop) malformed targets, do not silently repair.
- Behavior change: a target missing/invalid `xyz_world` is dropped at ingest
  rather than silently mis-sorted then rejected after selection. Net effect on a
  nav request for such a label is the same ("no_target"), just decided earlier.

## How to Demo

`pytest test/test_nav_manager.py test/test_nav_manager_pure.py` green, including a
new regression test that a target missing `xyz_world` is dropped by `on_targets`.
