# F24 — Remove Periodic Map Save from slam_manager

**Priority**: Medium
**Done:** no
**Tasks File Created:** yes (TF24)
**Tests Written:** no
**Test Passing:** no
**Description**: Remove the every-`m`-minutes periodic map save in `slam_manager_node.py`
(the `save_period_sec` timer added in F16). The periodic save serialized the modern
posegraph AND exported the legacy PGM/YAML on every tick; both go away. Map
persistence is left to the two event-driven saves that remain: **first-map** (once,
when slam_toolbox first publishes `/map`) and **shutdown** (synchronous final save on
clean lifecycle shutdown). Both remaining saves still write modern + legacy (subject
to `export_legacy_map`).

## Motivation

Periodic serialization is redundant with the shutdown save for normal runs and adds
recurring service-call load (serialize + legacy map_saver) during mapping. The
first-map save covers "something on disk early"; the shutdown save covers "final
state". Partly reverses F16 (periodic save + legacy export), keeping F16's legacy
export at the remaining save points.

## Scope

- `dome_nav/slam_manager_node.py` — remove `DEFAULT_SAVE_PERIOD_SEC`, the
  `save_period_sec` param, the `save_timer` (create in `on_activate`, teardown in
  `on_deactivate`/`destroy_entities`), and the `periodic_save` method. `on_activate`/
  `on_deactivate` keep only the lifecycle super-calls.
- `test/test_slam_manager.py` — drop `test_default_save_period_is_120`,
  `test_activate_starts_save_timer`, `test_deactivate_stops_save_timer`; add a
  regression test that activate creates no timer.
- Launch: drop the `save_period_sec: 60.0` overrides in `sim_nav_full.launch.py` and
  `sim_explore.launch.py` (setting a now-undeclared param would error at runtime).

## Constraints

- First-map save and shutdown sync save must be unchanged.
- `export_legacy_map` behavior at the remaining save points is untouched.

## How to Demo

1. Launch a mapping stack; watch logs — no "Pose graph saved" lines during steady
   mapping (only one at first map).
2. Clean shutdown → one final "Pose graph saved" + legacy export.
