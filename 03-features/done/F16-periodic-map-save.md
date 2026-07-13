# F16 — Periodic Map Save with Legacy Format Export

**Priority**: Medium
**Done:** yes
**Tasks File Created:** yes
**Tests Written:** yes
**Test Passing:** yes
**Description**: Save the slam_toolbox map automatically every 2 minutes during
exploration, and additionally export a legacy-format map (PNG occupancy image +
YAML metadata) compatible with Nav2's `map_server` / `nav2_map_server`. The
periodic save guards against losing a built map if the robot crashes or the
session is interrupted. The legacy export allows the saved map to be used
immediately for Mode B (AMCL localization) without a separate conversion step.

## Scope

- `dome_nav/slam_manager_node.py` — the periodic slam save already exists
  (`save_period_sec` ROS parameter, default 30 s, set to 120 s in sim launch).
  Change the default to 120 s (2 minutes) so real-robot launches inherit it without
  needing a launch-file override. No new timer needed.
- `dome_nav/slam_manager_node.py` — after each periodic slam_toolbox save
  (`.posegraph` + `.data`), also call slam_toolbox's `/slam_toolbox/save_map`
  service to export the legacy PNG + YAML pair at the same path prefix. This
  service already exists in slam_toolbox (`slam_toolbox_msgs/srv/SaveMap`); the
  node just needs a service client for it.
- The legacy files are written as `<map_name>.pgm` + `<map_name>.yaml` alongside
  the existing `<map_name>.posegraph` / `<map_name>.data` files in
  `~/.dome/slam_maps/`.
- New ROS parameter `export_legacy_map: bool = True` on `slam_manager_node` —
  set False to skip the PNG/YAML export (e.g. if slam_toolbox version does not
  ship the SaveMap service).
- Launch files — no changes needed; `save_period_sec` default change is picked
  up automatically.

## Constraints

- The `/slam_toolbox/save_map` call is best-effort: if it fails (service
  unavailable, slam not yet initialised), log a warning and continue — do not
  crash or block the periodic save of `.posegraph`/`.data`.
- The legacy export runs after the slam_toolbox serialisation call completes,
  not concurrently.
- No new Python dependencies; slam_toolbox_msgs is already a declared dependency.
- The change to `save_period_sec` default (30 → 120 s) must not break the
  existing `test_save_period_sec_default` test if one exists — update it.

## How to Demo

**Setup**: `bl dome_nav robot_explore.launch.py --map_name f16test` (or sim
equivalent). Let robot explore for at least 2 minutes.

**Steps**:
1. After 2 minutes observe log: `Saving map f16test` + `Exported legacy map`.
2. `ls ~/.dome/slam_maps/f16test*` — confirm `.posegraph`, `.data`, `.pgm`,
   `.yaml` all present.
3. Load `.yaml` into `map_server`: `ros2 run nav2_map_server map_server
   --ros-args -p yaml_filename:=~/.dome/slam_maps/f16test.yaml` — confirm
   `/map` publishes the occupancy grid without errors.

**Expected output**: four files written every 2 minutes; legacy YAML/PGM loadable
directly by `map_server` for a Mode B localization run with no manual conversion.
