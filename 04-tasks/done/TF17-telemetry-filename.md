# TF17 — Telemetry File Naming for F17

## T01 — Rename TelemetryWriter to use map_name + date
**Status**: done
**Description**: Changed `TelemetryWriter.__init__` to accept `map_name: str = "session"`.
Filename built as `e<map_name><dd-mmm>.json` (e.g. `ebasement110-jul.json`).
Collision avoidance: appends `-2`, `-3`, etc. if file already exists.
Removed `next_run_number()` and `exp-(\d{4})` pattern.
**Test**: `test_telemetry_filename_*` in `test_pluggable_explore_manager_node.py`.

## T02 — Pass map_name from pluggable_explore_manager_node
**Status**: done
**Description**: `TelemetryWriter(self.get_logger().info, map_name=self.map_name)` — `self.map_name` already available from the `map_name` ROS parameter.
**Test**: Covered by existing mock in `test_pluggable_explore_manager_node.py`.

## T03 — Tests
**Status**: done
**Description**: Pure tests for `build_telemetry_filename()`:
- basic name + date format
- invalid chars stripped/replaced
- collision suffix `-2`, `-3`
- default `map_name="session"` fallback

## T04 — Update feature file and current.md
**Status**: done
