# F17 — Telemetry File Naming: session name + day/month

**Priority**: Low
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Rename telemetry files from the sequential `exp-NNNN.json` scheme
to `e<map_name><dd-mmm>.json` (e.g. `efri010-jul.json`). Makes files human-readable
at a glance without opening them. When the same map name is used more than once on
the same day, append a numeric suffix to avoid overwriting
(e.g. `efri010-jul-2.json`).

## Scope

- `dome_nav/explore_telemetry.py` — change `TelemetryWriter.__init__` to accept
  `map_name: str` and build the filename from it plus the current wall-clock date
  (`datetime.now()` formatted as `%d-%b` lowercased, e.g. `10-jul`).
  Strip or replace characters in `map_name` that are invalid in filenames
  (spaces, slashes → `_`; max 32 chars). Collision avoidance: if
  `e<map_name><dd-mmm>.json` already exists, try `e<map_name><dd-mmm>-2.json`,
  `-3`, etc. until a free slot is found.
- Remove `next_run_number()` and the `exp-(\d{4})` pattern — no longer needed.
- `dome_nav/pluggable_explore_manager_node.py` — pass `self.map_name` to
  `TelemetryWriter(log_fn, map_name=self.map_name)`. Already available as
  `self.map_name` from the `map_name` ROS parameter.

## Constraints

- Old `exp-NNNN.json` files in `~/.dome/telemetry/` are left untouched — no
  migration required, they coexist.
- `TelemetryWriter` signature change is additive: `map_name` defaults to
  `"session"` so any other caller that doesn't pass it gets `esessionmmm-yy.json`
  rather than crashing.
- `datetime.now()` not `time.monotonic()` — filename needs wall-clock day/month.
  The `ts` field inside each JSONL record stays as `time.monotonic()` (unchanged).

## Also: dome_control hardware telemetry CSV rename

dome_control generates `telemetryDDMMYY.csv` (UPS/OAK-D/Pi system health, continuous
sampling). Rename to `t<dd-mmm>.csv` (e.g. `t10-jul.csv`). No session name — date
only. One file per calendar day; collision suffix (`t10-jul-2.csv`) if the writer
restarts on the same day.

This change lives in the **dome_control** package, not dome_nav. Add to dome_control
scope when implementing: find the CSV writer, apply the same `%d-%b` strftime pattern
(lowercased), same suffix-on-collision logic.

## How to Demo

**Steps**:
1. `bl dome_nav robot_explore.launch.py --map_name basement1`
2. Publish `exploration_start`, let one goal complete, stop.
3. `ls ~/.dome/telemetry/ebasement1*.json` — confirm file exists with correct name.
4. Run again same day → `ebasement110-jul-2.json` created, original untouched.

**Expected output**: telemetry files named `e<map_name><dd-mmm>.json`; no
sequential counter; re-runs same day increment suffix rather than overwrite.
