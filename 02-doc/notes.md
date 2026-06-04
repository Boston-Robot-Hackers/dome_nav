# dome_nav — Project Notes

## Architecture decisions

- slam_toolbox `online_async` mode chosen over `localization` — allows map to grow
  as robot explores new rooms. Switch to `localization` only once map is stable.
- `map_start_at_dock: true` — robot must always boot at same physical origin.
  If this assumption breaks, switch to a manual initial pose estimate via RViz or
  a `/initialpose` publisher.
- AMCL disabled — slam_toolbox owns the `map → odom` transform. AMCL and
  slam_toolbox cannot coexist as both publish the same TF edge.
- Nav2 goal frame is `map` — requires slam_toolbox to be running before
  nav_manager_node sends any goals. Startup order: slam first, nav second.
- dome_vision WorldTracker should publish in `map` frame, not `odom`. One-line
  change in `semantic_map_node.py`: `target_frame = "map"`.

## Map persistence details

slam_toolbox serializes two files: `.posegraph` (pose graph nodes and edges) and
`.data` (raw scan data). Both required to resume. Occupancy grid PNG is separate
and only needed for Nav2 static layer on localization-only runs.

`map_file_name` in slam.yaml is hardcoded to `~/.dome/slam_map`. slam_toolbox
silently starts fresh if the files do not exist — safe to always include this param.

## Known risks

- First traversal of any new area has no loop closure — deduplication in WorldTracker
  is only as good as odometry during that pass.
- If robot does NOT start at the dock, stored object positions in `map` frame will be
  misaligned until slam_toolbox localizes. Add a check: hold WorldTracker updates
  until `map → odom` transform is non-identity.
- nav2 `recoveries_server` plugin name may differ across Nav2 versions (renamed to
  `behavior_server` in newer releases). Check at build time.
