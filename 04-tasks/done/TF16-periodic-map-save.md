# TF16 — Periodic Map Save with Legacy Export for F16

## T01 — Change DEFAULT_SAVE_PERIOD_SEC 60 → 120
**Status**: done

## T02 — Add export_legacy_map param + SaveMap service client
**Status**: done
**Description**: New ROS param `export_legacy_map: bool = True`. New `save_map_client`
for `/slam_toolbox/save_map` (slam_toolbox_msgs/srv/SaveMap). Created in `on_configure`,
cleaned up in `destroy_entities`. After each successful posegraph save, if
`export_legacy_map` is True, calls the SaveMap service with `name.data = map_persist_path`.
Best-effort: logs warning on failure, never crashes periodic save.

## T03 — Tests
**Status**: done
**Description**: Added to `test_slam_manager.py`:
- `test_default_save_period_is_120`
- `test_export_legacy_map_default_true`
- `test_configure_creates_save_map_client`
- `test_on_save_done_triggers_legacy_export`
- `test_on_save_done_skips_legacy_when_disabled`
- `test_on_save_done_skips_legacy_on_future_none` (posegraph failed → no legacy call)
- `test_legacy_save_service_unavailable_logs_warning`

## T04 — Update feature file and current.md
**Status**: done
