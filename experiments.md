# Experiment log — dome_nav navigation and CPU experiments

Consolidated log covering four investigations:
- **Bug 1**: velocity_smoother deadband froze the robot (SOLVED).
- **Bug 2**: robot won't move when it STARTS inside a local_costmap inflation zone
  (start-in-inflation deadlock).
- **Pi CPU campaign**: idle-CPU reductions on the Pi5 (C1/C2/C4 confirmed, C3 note).
- **Explorer fail-fast reselection**: abandon hopeless goals early instead of
  grinding to timeout (design notes).

---

## Harness and measurement (shared)

`launch/nav_experiment.launch.py` — slam_toolbox + Nav2 only, no explorer. Takes
both config yamls as args so each run swaps configs without code edits. Driver
stack (tf/laser/odom/base) runs separately.

```
bl dome_nav nav_experiment.launch.py \
    --slam_config <slam yaml> \
    --nav2_config <nav2 yaml>
```

Amended for explorer runs: add `--map_name <name>` to also start slam_manager +
explore_manager (params mirror robot_explore.launch.py). Without `--map_name` it
stays nav-only. Core `robot_explore.launch.py` untouched.

cmd_vel chain (Jazzy): controller -> `cmd_vel_nav` -> velocity_smoother ->
`cmd_vel_smoothed` -> collision_monitor -> `cmd_vel` (to base). Echo all three +
odom while a goal is active:
```
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
```

Protocol: start driver stack; launch harness with the experiment's two yamls;
wait for map + tf; send the SAME rviz nav goal each run (record the pose); watch
the three cmd_vel topics and note whether the robot physically moves.

To run robot_explore (not the harness) the package must be rebuilt:
`cd ~/ros2_ws && colcon build --packages-select dome_nav`.

---

## Bug 1 — velocity_smoother deadband froze the robot (SOLVED)

**Problem:** Nav2 sends angular `cmd_vel` of ~0.16 rad/s during navigation; the
robot did not move. The "robot can't turn below 0.5 rad/s" hardware-stiction
premise was suspected but turned out false.

### Conclusion table
| Exp | Config | Robot moves? |
|-----|--------|--------------|
| E0 | pure upstream (`nav2_bringup/params/nav2_params.yaml`) | yes |
| E1 | our full config (`config/nav2_params_explore_real.yaml`) | no |
| E2 | upstream + only deadband `[0.1,0,0.3]` (`experiments/E2_upstream_plus_deadband.yaml`) | no |
| E3 | our config with deadband `[0,0,0]` (`experiments/E3_our_no_deadband.yaml`) | yes |

E0 baseline used upstream slam (`mapper_params_online_async.yaml`) and upstream
nav2; path planned and robot moved fine, disproving the hardware turn-floor
theory. E1 reproduced the freeze with our config (controller logged a long series
of "Passing new path to controller" — plans fine, no execution progress). E2 was
verbatim upstream with the single change `deadband_velocity: [0.1,0,0.3]` and
froze — a SUFFICIENT cause. E3 was our full config with only deadband -> [0,0,0]
and moved — the deadband is the ONLY culprit.

**Root cause:** `velocity_smoother.deadband_velocity: [0.1, 0.0, 0.3]`. Nav2's
deadband ZEROS any command below the threshold (it does not round up). MPPI emits
normal small angular commands (~0.16 rad/s) during path following; the 0.3 turn
deadband zeroed them, so the base received 0 and never moved.

**Fix (DONE):** set `deadband_velocity: [0.0, 0.0, 0.0]` in BOTH real configs
(`config/nav2_params_explore_real.yaml` and `config/nav2_params_real.yaml`). Also
DONE: the RotationShimController (added this session for the misdiagnosed bug) was
stripped from both real configs; FollowPath is back to raw MPPIController,
matching upstream.

---

## Bug 2 — start-in-inflation deadlock (IN PROGRESS)

**Problem:** when the robot STARTS inside a local_costmap inflation zone (footprint
in the colored band), it won't move. Separate from Bug 1. This is inherent Nav2
behavior — upstream fails identically.

### Root cause: three independent gates, all fire when starting in inflation
Confirmed by E4 (pure upstream, robot deliberately started in an inflation zone,
upstream `nav2_params.yaml`, inflation_radius 0.7) and its rerun E4b (2026-07-15).
The deadlock is NOT a single gate:

1. **Planner (start-occupied):** `GridBased failed to plan from (0.00,-0.00) to
   (0.04,1.70): "Failed to create plan with tolerance 0.5"`. (E4 with NavFn logged
   `GridBased (NavFn) failed to plan from (0,0): "Failed to create plan with
   tolerance 0.5"`.) Start cell cost >= 253 (inscribed) -> planner rejects the
   start. No path is ever produced.
2. **backup (behavior_server internal collision check):** `Collision Ahead -
   Exiting DriveOnHeading` -> `backup failed`. This is behavior_server's OWN
   internal collision check, SEPARATE from collision_monitor. Backup self-aborts;
   disabling collision_monitor will NOT free it.
3. **spin (collision_monitor / FootprintApproach):** `collision_monitor: Robot to
   approach for 1.2s away from collision` -> cmd_vel scaled ~0 -> spin times out at
   10s -> `spin failed`. Only THIS gate is collision_monitor.

Then `wait`, loop forever.

**collision_monitor is the downstream master gate (measured during E4):**
`cmd_vel_nav` = z 1.0 / x 0.25 (nav2 IS commanding motion — spin recovery + drive),
but `cmd_vel` NEVER exceeds 0. FootprintApproach (action "approach") scales velocity
to 0 whenever the footprint is already in the inflated/lethal zone
(time-to-collision ~= 0 for any motion), freezing ALL motion including recoveries.
It sits downstream of both planner and controller, so it blocks escape regardless
of the planner (Smac vs NavFn). But disabling it frees only gate 3; gates 1 and 2
still fire.

### What makes a start cell lethal (costmap cost 0-254)
| Value | Name | Meaning |
|-------|------|---------|
| 254 | LETHAL | actual obstacle cell |
| 253 | INSCRIBED | center within inscribed radius -> footprint collides at ANY heading |
| 128-252 | inflated | may collide depending on heading |
| 0 | free | |

Planner rejects the start when cost >= 253. Trigger: robot **center within
`robot_radius` (0.15 m) of a real obstacle** (inscribed radius ~0.164 m). This 253
ring is set by robot_radius / footprint, NOT inflation_radius — lowering
inflation_radius (0.25/0.3) does nothing to the 253 ring. Escape only when the
center is > 0.15 m from the obstacle. Cannot shrink below the true robot radius
without risking real collision.

### Live costmap probe (2026-07-15) — robot IS on a lethal cell
Probed `/global_costmap/costmap_raw` + `/local_costmap/costmap_raw` at robot pose
(tf map->base_footprint = (0.001,-0.000)), res 0.05:
- **GLOBAL (planner):** robot cell cost = **253** (inscribed). A true 254 cell
  0.05 m away. Entire 5x5 around robot >= 253. -> planner start-occupied proven.
  Robot on/against a MAPPED wall (global = static_layer only).
- **LOCAL (backup/spin):** robot cell = 230 (sub-lethal); nearest 253 at 0.05 m
  directly -y. Live lidar sees an obstacle one cell behind (local = voxel_layer).
- **Escape geometry:** lethal is -x/-y; costs drop toward +y/+x
  (253->234->230->201->173). Free direction = +y/+x. Backup drives -x (default
  reverse) INTO the 254 -> "Collision Ahead". Spin doesn't translate. No stock
  recovery follows the cost gradient, so the free +y/+x is never used. MPPI would
  use it but only runs with a plan, and the planner made none. True deadlock.

### Which costmap sees which obstacle (confirmed from configs)
| Costmap | Active plugins | "Obstacle" source |
|---------|----------------|-------------------|
| global (planner) | static_layer + inflation | mapped walls only (obstacle_layer dropped) |
| local (backup/spin/MPPI) | voxel_layer + inflation | live lidar hits only (static not in plugins) |

### Fix directions (bug 2)
- Prevent starting in inflation (approach/dock behavior, or manual nudge).
- Custom escape recovery that bypasses BOTH collision_monitor AND the
  DriveOnHeading collision check (drive blind for a short backup).
- robot_radius is the only lever on the lethal ring and can't go below robot size.

### No-code options for the dead-zone scenario (ranked)
1. Manual teleop nudge +x/+y ~0.2 m out of the 253 ring, then nav. Most reliable.
2. Don't start in inflation (place robot >= 0.2 m from walls at launch).
3. Shrink robot_radius toward true robot size (only config lever on 253 ring; risk).
4. Fix/re-map if the static wall is stale/thick (global has no live-scan layer).
5. AssistedTeleop stock behavior (config + joystick).

NOT viable no-code: inflation_radius (irrelevant to 253 ring), collision_monitor
off (frees only spin), backup heading (fixed reverse), gradient escape (no stock).

### Pending experiments

#### E5 — our fixed config with collision_monitor FootprintApproach DISABLED, start in inflation
- **nav2_config:** `experiments/E5_no_collision_monitor.yaml` = current fixed
  config with `collision_monitor.FootprintApproach.enabled: false`.
- **Command:**
  ```
  bl dome_nav nav_experiment.launch.py \
      --slam_config /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml \
      --nav2_config /home/pitosalas/ros2_ws/src/dome_nav/experiments/E5_no_collision_monitor.yaml
  ```
- **Hypothesis:** collision_monitor is the deadlock gate. With it off, cmd_vel
  passes through and the robot can spin/back out of inflation. Watch cmd_vel_nav
  vs cmd_vel; does the robot escape the colored zone?
- **Caveat (from E4b):** disabling FootprintApproach frees only gate 3. Gate 1
  (planner start-occupied) and gate 2 (DriveOnHeading self-abort) still fire; E5
  likely still deadlocks unless the start cell escapes the >=253 lethal ring.
- **Result:** _(pending)_

#### E6 — upstream + surgical time_before_collision 0.5, with explorer
- **nav2_config:** `experiments/E6_upstream_tbc05.yaml` = verbatim upstream
  nav2_params.yaml with ONE change: FootprintApproach
  `time_before_collision: 1.2 -> 0.5`. Rationale: robot is slow; 1.2 s lookahead
  over-brakes. Shorter horizon = collision_monitor scales velocity less
  aggressively, may let normal motion through while still guarding.
- **Command:**
  ```
  bl dome_nav nav_experiment.launch.py \
      --slam_config /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml \
      --nav2_config /home/pitosalas/ros2_ws/src/dome_nav/experiments/E6_upstream_tbc05.yaml \
      --map_name <name>
  ```
- **Hypothesis:** shorter approach horizon relaxes the collision_monitor gate
  during normal explore driving without disabling it. Does NOT fix the
  start-in-inflation deadlock (planner + backup gates unaffected). Watch cmd_vel
  vs cmd_vel_smoothed during explore; smoother driving? any new collisions?
  explorer reaching frontiers?
- **Result:** _(pending)_

#### E7 — OUR fixed config + time_before_collision 0.5, with explorer
- **nav2_config:** `experiments/E7_our_tbc05.yaml` = our full fixed explore config
  (`nav2_params_explore_real.yaml`: deadband `[0,0,0]`, shim stripped, all our
  overrides) with the single further change FootprintApproach
  `time_before_collision: 1.2 -> 0.5` (line 347). The "our config" counterpart to
  E6's "upstream config" — both isolate the tbc05 relaxation, E6 on upstream, E7 on
  ours, so the pair separates "tbc05 helps" from "our overrides interfere".
- **Command:**
  ```
  bl dome_nav nav_experiment.launch.py \
      --slam_config /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml \
      --nav2_config /home/pitosalas/ros2_ws/src/dome_nav/experiments/E7_our_tbc05.yaml \
      --map_name <name>
  ```
- **Hypothesis:** same as E6 (shorter approach horizon relaxes collision_monitor
  during normal explore driving without disabling it), but on OUR config to confirm
  the win survives our overrides. Does NOT fix the start-in-inflation deadlock.
- **Result:** _(pending)_

---

## Pi CPU campaign (separate from nav bugs)

Symptom: every node burns CPU even with no goal / no explore start. On Pi5, load
avg spiked to 12 during nav-on-NOGO. Independent causes found + fixed.

### Net result (idle, same state, real config)
| Metric | Production baseline | All fixes (C1+C2+C4) |
|-----------------------|---------------------|----------------------|
| load avg (1m) | 5.46 | ~2 |
| idle % | ~68% | ~84% |
| explore_manager | 13% (pre-C1) | <2% (off list) |
| lifecycle_manager | 14% | 7.6% |
| route/waypoint/docking| ~21% running | gone |

Irreducible remainder = Nav2 C++ servers ~5-7% each (costmap update loops + bond).
Production `robot_explore.launch.py` still on upstream nav2_bringup (C2 not ported
by choice); C1+C4 (explorer node) apply to production too once rebuilt.

### C1 — explore_manager idle at 10-20% CPU (FIXED)
- **Cause:** node held standing subscriptions to `/map`, `/global_costmap/costmap`,
  `/local_costmap/costmap`. rclpy deserializes the FULL OccupancyGrid on every
  publish BEFORE the callback runs; the callbacks just stored a ref. Big latched
  grids * several Hz * Python = 10-20% burned while idle (explore not even started).
- **Confirmed:** all three publishers are RELIABLE + TRANSIENT_LOCAL (latched) via
  `ros2 topic info -v`. Latched => last grid available on demand instantly.
- **Fix:** removed the three standing subs + their callbacks. Added `fetch_grid`
  using `rclpy.wait_for_message` with a matching latched QoS; map + global costmap
  fetched in `explore_tick` only when about to pick a frontier (state==exploring,
  no active goal), local costmap fetched lazily in `dump_failure_diagnostics`.
  `find_and_send_frontier` still reads `self.latest_*`, so unit tests unchanged
  (33 pass; 1 pre-existing unrelated `min_frontier_size` default-mismatch failure).
  File: `dome_nav/pluggable_explore_manager_node.py`.
- **Result:** explore_manager 13% -> 8.9%. Residual is the TF listener (`/tf` at
  30-50Hz, Python) — see C4.
- **Key rule:** an ACTIVE node's standing sub always pays full deserialization; no
  QoS "throttle by time" exists. Only ways to cut it: don't subscribe (lazy/
  on-demand `wait_for_message`), or a C++ `topic_tools throttle` republisher.

### C2 — unused Nav2 servers running ACTIVE (FIXED in harness)
- **Cause:** `nav2_bringup navigation_launch.py` hardcodes route_server,
  waypoint_follower, docking_server into `lifecycle_nodes`. They boot ACTIVE and
  each idles ~7% (bond heartbeat 10Hz + per-node DDS discovery/liveliness +
  standing TF/subs, e.g. docking's TF listener) REGARDLESS of whether their action
  is ever called. dome_nav uses none: explorer sends navigate_to_pose, never
  routes/waypoints/docks. lifecycle_manager topped 14% servicing all bonds.
- **Key rule:** an ACTIVE lifecycle node is a full spinning ROS node (executor +
  bond + DDS participant + standing subs). Uncalled action != idle process. Can't
  make it stop spinning via yaml — yaml only sets params on nodes the LAUNCH
  starts. Must drop it from the launch node list.
- **Fix:** `launch/nav2_experiment_navigation.launch.py` = faithful fork of
  upstream navigation_launch.py with route_server + waypoint_follower +
  docking_server removed from `lifecycle_nodes` and both node paths. Everything
  else identical (params_file, cmd_vel_nav remaps, composition path).
  `nav_experiment.launch.py` now includes the local trimmed launch instead of
  nav2_bringup. Production `robot_explore.launch.py` still on upstream — trim it
  too once the harness confirms the win. (If a BT errors "route_server/
  waypoint_follower not found" restore that one.)
- **Result:** CONFIRMED in harness (nav_experiment.launch.py, real explore config,
  map oi24). docking/route/waypoint gone from `ros2 node list`. Idle top: load avg
  5.46 -> 1.07; idle 68% -> 81.7%; lifecycle_manager 14% -> 7.3% (halved, fewer
  bonds); every server -2-3%. ~15% total CPU reclaimed. Production
  robot_explore.launch.py intentionally left on upstream nav2_bringup (untrimmed).
  Remaining top proc = explore_manager 8.3% (TF-listener residual, see C4).

### C3 — nav-on-NOGO CPU spike is Bug 2's deadlock, NOT a new bug
- During nav with the robot on a lethal/NOGO cell, load hit 12. This is the Bug-2
  deadlock busy-looping: planner retries the failed plan at
  `expected_planner_frequency` 20Hz (every plan fails, start occupied) +
  bt_navigator recovery loop at `cycle_frequency` 10Hz + behavior_server firing
  spin/backup that collision_monitor gates. Symptom of Bug 2, not separate. Real
  fix = escape the NOGO cell (see Bug-2 fix directions). Lowering
  expected_planner_frequency only masks it.

### C4 — explore_manager TF-listener residual (~8%) (FIXED)
- After C1, explore_manager still ~8% idle. Grid subs gone; remaining cost is the
  tf2_ros TransformListener deserializing the full `/tf` stream (30-50Hz, Python)
  plus `robot_xy_in_map` TF lookups each 1Hz tick. Only lookup used is
  map->base_footprint. Measured `ros2 topic hz /tf` ~= 40Hz (tf_static none).
- **Fix (implemented):** lazy TF listener, same pattern as C1 grids.
  TransformListener created in `start_tf` on exploration_start, torn down in
  `stop_tf` on stop/done (destroys the /tf + /tf_static subs it registered).
  `robot_xy_in_map` returns None when no buffer; `start_xy` captured lazily on the
  first tick TF is ready (buffer empty right after start). Idle node holds NO TF
  listener -> deserializes no /tf. During active explore the 40Hz cost returns
  (acceptable; nav is running). Tests: 33 pass, same 1 pre-existing unrelated
  failure.
- **Result:** CONFIRMED on hardware (harness, real explore config, map oi24).
  explore_manager across full lifecycle:
    - idle before start: <2.3% (off top list; was 8.3%)
    - exploring: 21.6% (TF listener 40Hz back + per-goal grid fetches)
    - stopped/idle after: <2.0% (off top list; ~5% seen transiently mid-teardown)
  Confirms lazy TF: sub created on start, torn down on stop, idle deserializes no
  /tf. Active-explore 21.6% > idle-lazy prediction because the on-demand grid
  fetches (wait_for_message deserializes map + global costmap) + 1Hz markers add on
  top of TF while exploring; acceptable (controller MPPI ~26% dominates nav).
  Further active-explore lever if ever needed: tf2 BufferClient + a C++
  buffer_server moves the 40Hz TF buffering off the explorer entirely. Rejected
  /pose as a TF replacement: slam_toolbox /pose was silent for 10s while
  stationary (gated by scan-match / map_update_interval 10s) -> too stale/irregular.

---

## Explorer fail-fast target reselection (design notes)

This is about the EXPLORER's response to navigation failure: notice failure early
and pick a different target instead of grinding on a goal that has no hope.

**Problem:** the explorer keeps a goal active until it succeeds, aborts, or hits
the 25s `GOAL_TIMEOUT_S`. When the robot is wedged (collision_monitor gate /
start-occupied deadlock — see Bug 2), that means ~25s of sitting still per hopeless
goal before moving on. We want to abandon fast and reselect.

### Two distinct failure cases (different detection + response)

#### Case A — no suitable targets
The frontier selector returns nothing sendable this tick.
- Detected in `find_and_send_frontier` -> `handle_no_frontier`.
- Today: increments `no_frontier_count`, declares `done` at `NO_FRONTIER_PATIENCE`
  (14). This CONFLATES two situations:
  - **raw clusters == 0** (`algorithm.latest_clusters` empty) -> genuinely fully
    explored -> `done` is correct.
  - **raw clusters > 0 but all filtered / blacklisted / outside costmap /
    unreachable** -> NOT done, we are BLOCKED, not finished. Declaring done here
    ends exploration prematurely.
- Proposed response for the blocked sub-case: before giving up, age-out the oldest
  blacklist entries (targets blacklisted early may now be reachable as the
  map/costmap grew) and/or relax a filter, with a hard cap so it can't loop forever.
  Only declare `done` when raw clusters == 0.
- [VERIFIED] The signal exists: `algorithm.latest_clusters`
  (frontier_algorithm.py:29) is the RAW clusters before size/blacklist/dist
  filtering, and handle_no_frontier already logs `raw_clusters=len(...)`. Only the
  DECISION conflates the two cases (patience declares done regardless of raw count).
  Fix is pure node logic.

#### Case B — target exists but robot is not moving (fail-fast stuck detection)
Goal accepted, but no forward progress (the deadlock/gate case).
- Today only `GOAL_TIMEOUT_S = 25s` catches it. Too slow.
- Add a NO-PROGRESS monitor while `has_active_goal`. Data already on hand:
  `robot_xy_in_map()`, `current_goal_xy`, `goal_start_time`.
- Track:
  - `best_dist_to_goal` — smallest distance-to-goal seen this goal.
  - `last_progress_xy`, `last_progress_time`.
- Each tick while active:
  - `d = dist(robot, goal)`; `moved = dist(robot, last_progress_xy)`.
  - PROGRESS if `d < best_dist - PROGRESS_EPS` OR `moved > MOVE_EPS`: update
    best_dist / last_progress_xy / last_progress_time.
  - else if `now - last_progress_time > STUCK_T`: declare STUCK.
- On STUCK: `cancel_goal_async`, blacklist target WITH radius, clear active goal,
  reselect next tick. ~4x faster than the 25s timeout.
- Keep `GOAL_TIMEOUT_S` as the hard cap (covers slow-but-not-stuck edge cases).

Proposed constants (tune on hardware):
| Name | Start value | Meaning |
|---------------|-------------|---------|
| STUCK_T | 6-8 s | no-progress window before abandon |
| MOVE_EPS | 0.05 m | min translation counted as progress |
| PROGRESS_EPS | 0.10 m | min distance-to-goal drop counted as progress |

Edge case to guard: final in-place rotation near the goal makes ~0 translation but
IS progress; distance-to-goal barely changes too. Mitigate: only run stuck
detection while `d` is above the goal tolerance (i.e. still en route, not doing the
final align), or treat "within goal tolerance" as success-imminent.

### Blacklist must be a REGION, not a point  [VERIFIED — already works]
`ExploreParams.blacklist_radius = 0.5` IS enforced on the live path: next_goal ->
pick_best_frontier -> best_cell_in_cluster (frontier_explorer.py:178) excludes any
frontier cell within `blacklist_radius` of ANY blacklisted point. So abandoning a
target already suppresses its neighborhood, not just the exact XY. Case B
stuck-abandon needs no algorithm change — just `blacklist.add(target)`. Not yet
supported: a per-cause radius (wider for controller/collision failures than planner
"no path"). `br` is a single param; threading a per-add radius is a small extra if
we want it.

### Bonus: classify by Nav2 error code (already captured)
`on_goal_result` already reads `result.error_code` / `error_msg` on ABORTED
(passed to `dump_failure_diagnostics`). Use it to pick the response:
- **planner error** (no valid path / start occupied) -> target unreachable FROM
  HERE -> blacklist point, next.
- **controller / collision error** -> robot cannot move out -> blacklist point +
  wider region, next.
Different blacklist radius per cause.

### Open questions / to decide
- Should a STUCK abandon also trigger a one-shot escape nudge (blind short backup)
  before reselecting, or purely reselect? (Escape recovery is a Bug-2 topic; keep
  this experiment to detection+reselection, note the tie-in.)
- Age-out policy for blacklist (time-based? map-growth-based?) to recover from
  early false-negatives without thrashing.
- Metrics to log per abandon: reason (no_frontier / stuck / planner_err /
  controller_err), elapsed, robot_xy, goal_xy, blacklist size — most already in
  telemetry; add a `reason` field for stuck.

### Runs
_(none yet — design notes only)_
