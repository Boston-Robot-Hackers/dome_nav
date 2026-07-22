# TF24 — Remove Periodic Map Save for F24

## T01 — Strip the periodic-save timer from slam_manager_node
**Status**: done
**Description**: Remove `DEFAULT_SAVE_PERIOD_SEC`, the `save_period_sec` param
(declare + read), `save_timer` (creation in `on_activate`, teardown in
`on_deactivate` and `destroy_entities`), and the `periodic_save` method. Keep the
first-map save (`on_map`) and shutdown sync save (`on_shutdown` → `save_map_sync`)
exactly as they are, both still routing through `save_map_async`/`save_map_sync` →
`on_save_done` → legacy export.
**Test**: `test/test_slam_manager.py` — drop the three timer/period tests; add
`test_activate_creates_no_timer`. Full suite green.

## T02 — Update docs/literate
**Status**: done
**Description**: `02-doc/current.md` key-params + F16 note reference the 120 s
periodic save; add a line that F24 removed it (saves now first-map + shutdown only).
Regenerate `01-literate/` for `slam_manager_node.py` before PR.
**Done**: current.md F16 refs annotated with F24 removal; literate
`02-slam_manager_node.md` (v3.4) already reflects the timer removal.
