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

## Future: costmap-based frontier exploration

The current `FrontierAlgorithm` reads `/map` (raw slam_toolbox occupancy grid,
unknown = -1). An alternative worth exploring is subscribing to Nav2's
`/global_costmap/costmap` instead.

Key differences:

- Inflation layer: free cells near obstacles are inflated to higher cost values
  (not occupied). Goals placed in the inflated zone will be reached but may be
  slightly blocked; Nav2's planner avoids them naturally. Using the costmap means
  the nudge (`goal_inset_m`) toward the robot may become unnecessary since frontier
  cells won't be on the raw obstacle boundary.
- Unknown encoding: costmap encodes unknown as 255 (uint8), not -1. A
  `CostmapFrontierAlgorithm` must treat 255 as unknown when finding frontier
  cells (free cells adjacent to 255 cells).
- Dynamic obstacles: global costmap incorporates the local costmap's dynamic
  obstacle layer, so transient obstacles (e.g., a person walking by) affect
  goal selection. Could be a benefit or a source of jitter.
- Potential benefit: exploration goals would naturally land in navigable space
  without any nudge or boundary-check workaround.

This fits cleanly into the pluggable design (F12): implement as a new class
`CostmapFrontierAlgorithm` in a new file, drop-in replacement for
`FrontierAlgorithm`, injected at node construction. No existing code changes.

## Future: path-aware frontier selection

The lidar scans continuously during transit, not just at the destination. By the
time the robot arrives at a frontier goal, cells along the entire path are already
uncovered. Three algorithmic improvements follow from this:

**1. Mid-navigation re-evaluation (implemented in `explorer_manager_node.py`)**
Run the frontier algorithm every tick even with an active goal. If the best frontier
has shifted more than `REDIRECT_THRESHOLD` (currently 1.5 m) from the current goal,
cancel and redirect. Implemented via `check_goal_redirect()` and the `is_redirecting`
flag (suppresses blacklisting on preemptive cancel). The pluggable design supports
this without touching `FrontierAlgorithm`.

**2. Adaptive goal distance (not yet implemented)**
Frontiers that fall within the path-scan corridor to the current goal will be
uncovered for free — they don't need to be directly targeted. The algorithm should
prefer farther frontiers when nearby ones lie along the expected travel path, to
avoid arriving at a destination that is already explored. Would require the algorithm
to know the planned path or at least the current heading.

**3. Directional continuity bonus (not yet implemented)**
A frontier roughly ahead of the robot's current heading costs less than one requiring
a detour, because path scanning along the current direction is a byproduct of travel.
A utility function that discounts frontiers in the current travel direction (they'll
be covered en route) and values frontiers off the current axis (genuinely new
territory) would improve coverage efficiency. Fits the pluggable design as a new
`DirectionalFrontierAlgorithm` class.

The nearest-frontier selection and blacklist logic remain correct. The nudge geometry
is unchanged. Only point 1 has been implemented.

## Known risks

- First traversal of any new area has no loop closure — deduplication in WorldTracker
  is only as good as odometry during that pass.
- If robot does NOT start at the dock, stored object positions in `map` frame will be
  misaligned until slam_toolbox localizes. Add a check: hold WorldTracker updates
  until `map → odom` transform is non-identity.
- nav2 `recoveries_server` plugin name may differ across Nav2 versions (renamed to
  `behavior_server` in newer releases). Check at build time.
