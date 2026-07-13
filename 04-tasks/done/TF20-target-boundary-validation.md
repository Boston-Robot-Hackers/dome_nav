# TF20 — Validate Nav Targets Once at the Boundary for F20

## T01 — Filter invalid targets in on_targets; trust downstream
\*\*Status\*\*: done
**Description**: Add `is_valid_target()` (dict + numeric `xyz_world` len>=2).
`on_targets()` stores only valid targets. `find_nearest_confirmed()` reads
`target["xyz_world"]` directly (drop the `[0,0,0]` default).
**Test**: pure regression test — `on_targets` with a target missing `xyz_world`
drops it; valid ones kept.

## T02 — Simplify navigate_to_object (drop re-check + coercion)
\*\*Status\*\*: done
**Description**: Remove the `if xyz is None` block and `float()` coercions;
`yaw_world` stays optional without `float()`.
**Test**: update `test_navigate_target_missing_xyz_world_*` to route through
`on_targets` (new contract: dropped at ingest → no_target).

## T03 — Update feature file and current.md
\*\*Status\*\*: done
