# Experiment log — "robot won't move / tiny cmd_vel" problem

## >>> RESUME HERE (session paused 2026-07-15, Pi restart) <<<
**Two bugs found:**
1. SOLVED: `velocity_smoother.deadband_velocity: [0.1,0,0.3]` zeroed MPPI's
   normal small commands -> robot froze. Fixed to `[0,0,0]` in BOTH real
   configs. Also stripped the RotationShimController (was a fix for this
   misdiagnosis) -> FollowPath back to raw MPPI. See E0-E3 + Conclusion.
2. IN PROGRESS: when robot STARTS inside a local_costmap inflation zone it
   won't move. Root = `collision_monitor` FootprintApproach scales cmd_vel to 0
   once footprint is in the inflated/lethal zone (proven: cmd_vel_nav z=1.0/
   x=0.25 during spin recovery, but cmd_vel stays 0). Also NavFn planner fails
   ("start occupied"). Deadlock: no plan + recovery blocked by collision_monitor.
   FootprintApproach IS in upstream too (E4 upstream froze identically).

**NEXT ACTION: run E5** (below) -- our config with collision_monitor
FootprintApproach disabled, start in inflation. Command:
```
bl dome_nav nav_experiment.launch.py \
    --slam_config /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml \
    --nav2_config /home/pitosalas/ros2_ws/src/dome_nav/experiments/E5_no_collision_monitor.yaml
```
Watch cmd_vel_nav vs cmd_vel; does robot escape the colored zone?

**Uncommitted work (not yet committed):**
- M config/nav2_params_explore_real.yaml  (deadband fix, shim removed, upstream audit)
- M config/nav2_params_real.yaml           (deadband fix, shim removed)
- ?? experiment.md                          (this file)
- ?? experiments/                           (E2/E3/E5 test configs)
- ?? launch/nav_experiment.launch.py        (experiment harness)
- D  current-once.md                        (pre-existing deletion, NOT ours)
Note: the deadband fix + shim removal are the keepers to commit once bug #2 is
resolved. To run robot_explore (not the harness) the package must be rebuilt:
`cd ~/ros2_ws && colcon build --packages-select dome_nav`.

---


## Problem
Nav2 sends angular `cmd_vel` of ~0.16 rad/s during navigation. The robot does
not move — below its physical turn floor (~0.5 rad/s). Goal: systematically
isolate whether the cause is our config, Nav2 behavior, or robot calibration.

## Harness
`launch/nav_experiment.launch.py` — slam_toolbox + Nav2 only, no explorer.
Takes both config yamls as args so each run swaps configs without code edits.
Driver stack (tf/laser/odom/base) runs separately.

```
bl dome_nav nav_experiment.launch.py \
    --slam_config <slam yaml> \
    --nav2_config <nav2 yaml>
```

## Measurement (same every run)
cmd_vel chain (Jazzy): controller -> `cmd_vel_nav` -> velocity_smoother ->
`cmd_vel_smoothed` -> collision_monitor -> `cmd_vel` (to base).

Echo all three + odom while a goal is active:
```
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
```

## Protocol
1. Start driver stack.
2. Launch harness with the experiment's two yamls.
3. Wait for map + tf.
4. Send the SAME rviz nav goal each run (record the pose).
5. Watch the three cmd_vel topics; note whether the robot physically moves.

---

## Runs

### E0 — pure upstream (baseline)
- **slam_config:** `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`
- **nav2_config:** `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`
- **Hypothesis:** upstream has deadband `[0,0,0]` (no zeroing) and raw MPPI. If the
  robot still won't move, the cause is the hardware turn floor, not our config.
  If it moves, our overrides (deadband/etc.) are implicated.
- **Note:** upstream uses NavfnPlanner (known "legal potential" bug) and
  inflation_radius 0.7 vs our small footprint — planning may fail for a
  different reason. Distinguish "failed to PLAN" from "planned but won't MOVE".
- **Result:** Path planned, robot moved with no problem. => Hardware turn-floor
  theory is FALSE — the robot executes small turns fine. Cause is in our config,
  not the robot and not Nav2 itself.

### E1 — upstream slam + OUR nav2 config
- **slam_config:** `/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml`
- **nav2_config:** `<pkg>/config/nav2_params_explore_real.yaml`
- **Hypothesis:** our nav2 config breaks it (E0 already cleared slam + Nav2).
  Prime suspect: `deadband_velocity: [0.1,0,0.3]` (upstream [0,0,0]) zeros turns
  <0.3 rad/s, killing the ~0.16 command. Confirm our config reproduces the bug.
- **Result:** Robot did NOT move; controller logged a long series of "Passing
  new path to controller" (plans fine, no execution progress -> replan loop).
  => our nav2 config reproduces the bug. Planning OK, execution dead.

### E2 — UPSTREAM nav2 + ONLY deadband changed
- **slam_config:** upstream mapper_params_online_async.yaml
- **nav2_config:** `experiments/E2_upstream_plus_deadband.yaml`
  = verbatim upstream with the single change `deadband_velocity: [0.1,0,0.3]`.
- **Hypothesis:** additive from known-good E0. If this one change breaks it,
  deadband is a sufficient cause. If it still moves, deadband is NOT the cause
  and something else in our config is.
- **Result:** Robot did NOT move. => Adding ONLY the deadband to a working
  upstream config breaks it. `deadband_velocity: [0.1,0,0.3]` is a SUFFICIENT
  cause: it zeros MPPI's normal small commands (0.16 < 0.3 floor). Root cause
  found.

### E3 — OUR full config with deadband removed ([0,0,0])
- **nav2_config:** `experiments/E3_our_no_deadband.yaml` = our explore_real
  config, deadband -> [0,0,0], everything else unchanged.
- **Hypothesis:** deadband is the ONLY culprit; our full config now works.
  If it moves, fix = drop deadband from the real configs. If still dead, there
  is a second breaker to find.
- **Result:** Robot MOVES. => deadband is the ONLY culprit.

---

## Conclusion
| Exp | Config | Robot moves? |
|-----|--------|--------------|
| E0 | pure upstream | yes |
| E1 | our full config | no |
| E2 | upstream + only deadband [0.1,0,0.3] | no |
| E3 | our config with deadband [0,0,0] | yes |

Root cause: `velocity_smoother.deadband_velocity: [0.1, 0.0, 0.3]`. Nav2's
deadband ZEROS any command below the threshold (it does not round up). MPPI
emits normal small angular commands (~0.16 rad/s) during path following; the
0.3 turn deadband zeroed them, so the base received 0 and never moved. The
"robot can't turn below 0.5 rad/s" stiction premise was false (E0 moved fine).

Fix: set `deadband_velocity: [0.0, 0.0, 0.0]` in the real configs.
Follow-up: DONE -- the RotationShimController (added this session for the
misdiagnosed bug) was stripped from both real configs; FollowPath is back to
raw MPPIController, matching upstream.

---

## Second problem: robot won't move when it STARTS inside a local_costmap
## inflation zone (footprint in the colored band). Separate from the deadband bug.

### E4 — pure upstream, robot deliberately started IN an inflation zone
- **slam_config:** upstream mapper_params_online_async.yaml
- **nav2_config:** upstream nav2_params.yaml (unchanged; inflation_radius 0.7)
- **Hypothesis:** if upstream ALSO can't move out of inflation, the entrapment
  is inherent to Nav2 (MPPI/collision_monitor), not our tuning -> fix must be
  behavioral (recovery/clearing), not just a smaller inflation_radius.
  If upstream DOES escape, our config is doing something extra that traps it.
- **Watch:** cmd_vel_nav vs cmd_vel (collision_monitor gate vs MPPI give-up);
  did it drive out of the colored zone?
- **Result:** Did NOT move -- but failure is at the PLANNER, not controller:
  `GridBased (NavFn) failed to plan from (0,0): "Failed to create plan with
  tolerance 0.5"`. Start cell is lethal/inscribed (robot within ~0.164 m
  inscribed radius of an obstacle). No plan -> recoveries fire -> spin times out
  (collision_monitor blocks rotation) -> wait -> deadlock. Entrapment is inherent
  Nav2 behavior; upstream fails the same way. inflation_radius is NOT the lever
  (inscribed 253-ring is set by footprint/robot_radius, not inflation_radius).

### Refinement: collision_monitor is the master gate
Measured during E4: `cmd_vel_nav` = z 1.0 / x 0.25 (nav2 IS commanding motion --
spin recovery + drive), but `cmd_vel` NEVER exceeds 0. => collision_monitor
(FootprintApproach, action "approach") scales velocity to 0 whenever the
footprint is already in the inflated/lethal zone (time-to-collision ~= 0 for any
motion). It freezes ALL motion including recoveries -> the robot can never
escape inflation. This gate sits downstream of both planner and controller, so
it blocks escape regardless of the planner (Smac vs NavFn).

### E5 — OUR fixed config with collision_monitor FootprintApproach DISABLED, start in inflation
- **nav2_config:** `experiments/E5_no_collision_monitor.yaml` = current fixed
  config with `collision_monitor.FootprintApproach.enabled: false`.
- **Hypothesis:** collision_monitor is the deadlock gate. With it off, cmd_vel
  passes through and the robot can spin/back out of inflation.
- **Watch:** cmd_vel now nonzero? robot escapes the colored zone?
- **Result:** _(pending)_

### E4b — upstream rerun (2026-07-15), start in inflation. Reproduces E4 exactly.
- **nav2_config:** upstream nav2_params.yaml
- **Result:** Deadlock confirmed. Log shows THREE independent gates, all fire
  when robot starts in inflation:
  1. **Planner**: `GridBased failed to plan from (0.00,-0.00) to (0.04,1.70):
     "Failed to create plan with tolerance 0.5"`. Start cell cost >= 253
     (inscribed) -> planner rejects start. No path ever produced.
  2. **backup**: `Collision Ahead - Exiting DriveOnHeading` -> `backup failed`.
     This is behavior_server's OWN internal collision check, SEPARATE from
     collision_monitor. Backup self-aborts; disabling collision_monitor will
     NOT free it.
  3. **spin**: `collision_monitor: Robot to approach for 1.2s away from
     collision` -> cmd_vel scaled ~0 -> spin times out at 10s -> `spin failed`.
     Only THIS gate is collision_monitor (FootprintApproach).
  Then `wait`, loop forever.
- **Key new finding:** the deadlock is not a single gate. E5 (disable
  FootprintApproach) frees only gate 3. Gate 1 (planner start-occupied) and
  gate 2 (DriveOnHeading self-abort) still fire. E5 likely still deadlocks
  unless the start cell escapes the >=253 lethal ring.

### What makes a start cell lethal (costmap cost 0-254)
| Value | Name | Meaning |
|-------|------|---------|
| 254 | LETHAL | actual obstacle cell |
| 253 | INSCRIBED | center within inscribed radius -> footprint collides at ANY heading |
| 128-252 | inflated | may collide depending on heading |
| 0 | free | |
Planner rejects start when cost >= 253. Trigger: robot **center within
`robot_radius` (0.15 m) of a real obstacle**. This ring is set by robot_radius,
NOT inflation_radius -- lowering inflation_radius (0.25/0.3) does nothing to the
253 ring. Escape only when center > 0.15 m from obstacle. Cannot shrink below
true robot radius without risking real collision.

### Fix directions (bug 2)
- Prevent starting in inflation (approach/dock behavior, or manual nudge).
- Custom escape recovery that bypasses BOTH collision_monitor AND the
  DriveOnHeading collision check (drive blind for a short backup).
- robot_radius is the only lever on the lethal ring and can't go below robot size.

### Live costmap probe (2026-07-15) — robot IS on a lethal cell
Probed `/global_costmap/costmap_raw` + `/local_costmap/costmap_raw` at robot
pose (tf map->base_footprint = (0.001,-0.000)). res 0.05.
- **GLOBAL (planner):** robot cell cost = **253** (inscribed). A true 254 cell
  0.05 m away. Entire 5x5 around robot >= 253. -> planner start-occupied proven.
  Robot on/against a MAPPED wall (global = static_layer only).
- **LOCAL (backup/spin):** robot cell = 230 (sub-lethal); nearest 253 at 0.05 m
  directly -y. Live lidar sees obstacle one cell behind (local = voxel_layer).
- **Escape geometry:** lethal is -x/-y; costs drop toward +y/+x
  (253->234->230->201->173). Free direction = +y/+x. Backup drives -x (default
  reverse) INTO the 254 -> "Collision Ahead". Spin doesn't translate. No stock
  recovery follows the cost gradient, so the free +y/+x is never used. MPPI
  would use it but only runs with a plan, and planner made none. True deadlock.

### Which costmap sees which obstacle (confirmed from configs)
| Costmap | Active plugins | "Obstacle" source |
|---------|----------------|-------------------|
| global (planner)     | static_layer + inflation | mapped walls only (obstacle_layer dropped) |
| local (backup/spin/MPPI) | voxel_layer + inflation | live lidar hits only (static not in plugins) |

### No-code options for the dead-zone scenario (ranked)
1. Manual teleop nudge +x/+y ~0.2 m out of the 253 ring, then nav. Most reliable.
2. Don't start in inflation (place robot >= 0.2 m from walls at launch).
3. Shrink robot_radius toward true robot size (only config lever on 253 ring; risk).
4. Fix/re-map if the static wall is stale/thick (global has no live-scan layer).
5. AssistedTeleop stock behavior (config + joystick).
NOT viable no-code: inflation_radius (irrelevant to 253 ring), collision_monitor
off (frees only spin), backup heading (fixed reverse), gradient escape (no stock).

---

## Phase 2: explore-node experiments (harness now includes explorer)
`launch/nav_experiment.launch.py` amended: add `--map_name <name>` to also start
slam_manager + explore_manager (params mirror robot_explore.launch.py). Without
--map_name it stays nav-only as before. Core robot_explore.launch.py untouched.

### E6 — upstream + surgical time_before_collision 0.5, with explorer
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
  during normal explore driving without disabling it. Does NOT fix the start-in-
  inflation deadlock (planner + backup gates unaffected).
- **Watch:** cmd_vel vs cmd_vel_smoothed during explore; smoother driving? any
  new collisions? explorer reaching frontiers?
- **Result:** _(pending)_

---

## Phase 3: CPU load on the Pi (separate from nav bugs)

Symptom: every node burns CPU even with no goal / no explore start. On Pi5,
load avg spiked to 12 during nav-on-NOGO. Two independent causes found + fixed.

### C1 — explore_manager idle at 10-20% CPU (FIXED)
- **Cause:** node held standing subscriptions to `/map`, `/global_costmap/costmap`,
  `/local_costmap/costmap`. rclpy deserializes the FULL OccupancyGrid on every
  publish BEFORE the callback runs; the callbacks just stored a ref. Big latched
  grids * several Hz * Python = 10-20% burned while idle (explore not even started).
- **Confirmed:** all three publishers are RELIABLE + TRANSIENT_LOCAL (latched)
  via `ros2 topic info -v`. Latched => last grid available on demand instantly.
- **Fix:** removed the three standing subs + their callbacks. Added `fetch_grid`
  using `rclpy.wait_for_message` with a matching latched QoS; map + global costmap
  fetched in `explore_tick` only when about to pick a frontier (state==exploring,
  no active goal), local costmap fetched lazily in `dump_failure_diagnostics`.
  Idle node now has NO grid subs -> ~0 grid deserialization. `find_and_send_frontier`
  still reads `self.latest_*`, so unit tests unchanged (33 pass; 1 pre-existing
  unrelated `min_frontier_size` default-mismatch failure).
- **Result:** explore_manager 13% -> 8.9%. Residual is the TF listener (`/tf` at
  30-50Hz, Python). File: `dome_nav/pluggable_explore_manager_node.py`.
- **Key rule:** an ACTIVE node's standing sub always pays full deserialization;
  no QoS "throttle by time" exists. Only ways to cut it: don't subscribe (lazy/
  on-demand `wait_for_message`), or a C++ `topic_tools throttle` republisher.

### C2 — unused Nav2 servers running ACTIVE (FIXED in harness)
- **Cause:** `nav2_bringup navigation_launch.py` hardcodes route_server,
  waypoint_follower, docking_server into `lifecycle_nodes`. They boot ACTIVE and
  each idles ~7% (bond heartbeat 10Hz + per-node DDS discovery/liveliness +
  standing TF/subs, e.g. docking's TF listener) REGARDLESS of whether their
  action is ever called. dome_nav uses none: explorer sends navigate_to_pose,
  never routes/waypoints/docks. lifecycle_manager topped 14% servicing all bonds.
- **Key rule:** an ACTIVE lifecycle node is a full spinning ROS node (executor +
  bond + DDS participant + standing subs). Uncalled action != idle process. Can't
  make it stop spinning via yaml -- yaml only sets params on nodes the LAUNCH
  starts. Must drop it from the launch node list.
- **Fix:** `launch/nav2_experiment_navigation.launch.py` = faithful fork of
  upstream navigation_launch.py with route_server + waypoint_follower +
  docking_server removed from `lifecycle_nodes` and both node paths. Everything
  else identical (params_file, cmd_vel_nav remaps, composition path).
  `nav_experiment.launch.py` now includes the local trimmed launch instead of
  nav2_bringup. Production `robot_explore.launch.py` still on upstream -- trim it
  too once the harness confirms the win.
- **Watch after rebuild:** `top` -- route_server/opennav_docking gone, ~14-20%
  back, lighter lifecycle_manager. Nav still works (no BT node calls a dropped
  server; if it errors "route_server/waypoint_follower not found" restore that one).
- **Result:** CONFIRMED in harness (nav_experiment.launch.py, real explore config,
  map oi24). docking/route/waypoint gone from `ros2 node list`. Idle top:
  load avg 5.46 -> 1.07; idle 68% -> 81.7%; lifecycle_manager 14% -> 7.3% (halved,
  fewer bonds); every server -2-3%. ~15% total CPU reclaimed. Production
  robot_explore.launch.py intentionally left on upstream nav2_bringup (untrimmed).
  Remaining top proc = explore_manager 8.3% (TF-listener residual, see C4).

### C4 — explore_manager TF-listener residual (~8%) (in progress)
- After C1, explore_manager still ~8% idle. Grid subs gone; remaining cost is the
  tf2_ros TransformListener deserializing the full `/tf` stream (30-50Hz, Python)
  plus `robot_xy_in_map` TF lookups each 1Hz tick. Only lookup used is
  map->base_footprint. Measured `ros2 topic hz /tf` ~= 40Hz (tf_static none).
- **Fix (implemented):** lazy TF listener, same pattern as C1 grids. TransformListener
  created in `start_tf` on exploration_start, torn down in `stop_tf` on stop/done
  (destroys the /tf + /tf_static subs it registered). `robot_xy_in_map` returns
  None when no buffer; `start_xy` captured lazily on the first tick TF is ready
  (buffer empty right after start). Idle node holds NO TF listener -> deserializes
  no /tf. During active explore the 40Hz cost returns (acceptable; nav is running).
  Tests: 33 pass, same 1 pre-existing unrelated failure.
- **Result:** CONFIRMED on hardware (harness, real explore config, map oi24).
  explore_manager across full lifecycle:
    idle before start : <2.3% (off top list; was 8.3%)
    exploring         : 21.6% (TF listener 40Hz back + per-goal grid fetches)
    stopped/idle after: <2.0% (off top list; ~5% seen transiently mid-teardown)
  Confirms lazy TF: sub created on start, torn down on stop, idle deserializes no
  /tf. Note active-explore 21.6% > idle-lazy prediction: the on-demand grid
  fetches (wait_for_message deserializes map + global costmap) + 1Hz markers add
  on top of TF while exploring; acceptable (controller MPPI ~26% dominates nav).
  Further active-explore lever if ever needed: tf2 BufferClient + a C++
  buffer_server moves the 40Hz TF buffering off the explorer entirely.
  Rejected /pose as a TF replacement: slam_toolbox /pose was silent for 10s while
  stationary (gated by scan-match / map_update_interval 10s) -> too stale/irregular.

### CPU campaign net result (idle, same state, real config)
| Metric                | Production baseline | All fixes (C1+C2+C4) |
|-----------------------|---------------------|----------------------|
| load avg (1m)         | 5.46                | ~2                   |
| idle %                | ~68%                | ~84%                 |
| explore_manager       | 13% (pre-C1)        | <2% (off list)       |
| lifecycle_manager     | 14%                 | 7.6%                 |
| route/waypoint/docking| ~21% running        | gone                 |
Irreducible remainder = Nav2 C++ servers ~5-7% each (costmap update loops + bond).
Production robot_explore.launch.py still on upstream nav2_bringup (C2 not ported
by choice); C1+C4 (explorer node) apply to production too once rebuilt.

### C3 — nav-on-NOGO CPU spike is bug 2's deadlock, NOT a new bug
- During nav with robot on a lethal/NOGO cell, load hit 12. This is the bug-2
  deadlock busy-looping: planner retries failed plan at `expected_planner_frequency`
  20Hz (every plan fails, start occupied) + bt_navigator recovery loop at
  `cycle_frequency` 10Hz + behavior_server firing spin/backup that collision_monitor
  gates. Symptom of bug 2, not separate. Real fix = escape the NOGO cell (see
  bug-2 fix directions). Lowering expected_planner_frequency only masks it.
