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

## Navigation investigation findings (migrated from deleted experiments.md)

`experiments.md` + `experiments/` were removed 2026-07-17; full run logs, conclusion
tables, and costmap probes are in git history (`git show 3d1b187:experiments.md`).
The durable findings:

### Bug 1 — velocity_smoother deadband froze the robot (SOLVED)
`velocity_smoother.deadband_velocity: [0.1, 0.0, 0.3]` ZEROS any command below the
threshold (it does not round up). MPPI emits normal small angular commands (~0.16
rad/s) during path following; the 0.3 turn deadband zeroed them, so the base got 0
and never moved. **Fix: `deadband_velocity: [0.0, 0.0, 0.0]`** (upstream's default) in
both real configs. The "robot can't turn below 0.5 rad/s hardware stiction" theory was
false — pure upstream moved fine.

### Bug 2 — start-in-inflation deadlock (UNSOLVED; inherent Nav2)
When the robot STARTS inside a local_costmap inflation zone it won't move. Not one
gate but **three independent ones, all firing**:
1. **Planner start-occupied** — start cell cost ≥ 253 (inscribed) → planner rejects
   the start, no path ever produced (`Failed to create plan with tolerance 0.5`).
2. **backup self-abort** — behavior_server's OWN internal DriveOnHeading collision
   check (`Collision Ahead - Exiting DriveOnHeading`), separate from collision_monitor.
3. **spin** — collision_monitor FootprintApproach scales cmd_vel to ~0.

Key facts:
- The 253 inscribed ring is set by **`robot_radius`** (0.15, inscribed ~0.164 m), NOT
  `inflation_radius` — lowering inflation does nothing to it. Escape only when the
  robot center is > 0.15 m from the obstacle. Can't shrink below true robot size.
- collision_monitor is the downstream master gate, but disabling it frees ONLY gate 3;
  gates 1 and 2 still fire. (E5 experiment, never run on hardware.)
- Global costmap (planner) = static_layer only → mapped walls. Local costmap
  (backup/spin/MPPI) = voxel_layer only → live lidar. No stock recovery follows the
  cost gradient out, so the free direction is never used.
- **Fixes (no code):** don't start in inflation (place ≥0.2 m off walls); manual teleop
  nudge out of the 253 ring, then resume. **Code fix would be** a custom escape recovery
  that blind-backs a short distance bypassing both collision checks.
- `time_before_collision` 1.2→0.5 (E6/E7) and collision_monitor-off (E5) are DESIGNED
  but **never run** — hypotheses, not results. F25 mini config carries the 0.5 tweak
  flagged UNVERIFIED.

### Pi CPU campaign (idle CPU reduction, CONFIRMED on hardware)
Every node burned CPU even with no goal. Independent causes, each fixed; net idle load
avg 5.46 → ~2, idle 68% → 84%.
- **C1** — `explorer_manager_node` held standing subs to `/map` + both costmaps; rclpy
  deserializes the FULL OccupancyGrid on every publish (10–20% idle). Fix: lazy
  on-demand `fetch_grid` via `wait_for_message` with latched QoS, only when about to
  pick a frontier. → 8.9%.
- **C2** — `nav2_bringup navigation_launch.py` boots route_server + waypoint_follower +
  docking_server ACTIVE (~7% each, bond heartbeat + DDS) even though dome_nav calls
  none. Fix: `nav2_experiment_navigation.launch.py` drops them from `lifecycle_nodes`.
  → load 5.46→1.07. **Production `robot_explore.launch.py` still on upstream nav2_bringup
  (untrimmed by choice).**
- **C3** — nav-on-NOGO CPU spike (load 12) is Bug 2's deadlock busy-looping (planner
  retries at 20 Hz + recovery loop), NOT a separate bug.
- **C4** — `explorer_manager_node` TF listener deserializes `/tf` (~40 Hz, Python) ~8%
  idle. Fix: lazy TransformListener created on `exploration_start`, torn down on stop.
  Idle node holds no listener → <2%.

Rule of thumb from C1/C2/C4: an ACTIVE ROS node's standing subscription always pays
full deserialization; no QoS throttles by time. Cut it only by not subscribing
(lazy/on-demand `wait_for_message`) or dropping the node from the launch.

## Known risks

- First traversal of any new area has no loop closure — deduplication in WorldTracker
  is only as good as odometry during that pass.
- If robot does NOT start at the dock, stored object positions in `map` frame will be
  misaligned until slam_toolbox localizes. Add a check: hold WorldTracker updates
  until `map → odom` transform is non-identity.
- nav2 `recoveries_server` plugin name may differ across Nav2 versions (renamed to
  `behavior_server` in newer releases). Check at build time.
