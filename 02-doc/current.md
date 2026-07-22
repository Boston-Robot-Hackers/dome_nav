# dome_nav — Current Session Handoff

Concise cold-start orientation. Detailed history lives in git log and the
`04-tasks/` files — do **not** re-narrate it here.

**Date:** 2026-07-22 · **Branch:** main

## This session (2026-07-22) — collision_monitor gate probe (preliminary)

Goal: pin down **which gate** stops the wedged robot (F29 mandatory probe).

- `ros2 topic echo /collision_monitor_state` while wedged shows alternating
  `action_type: 3` / `polygon_name: FootprintApproach` ↔ `action_type: 0` (empty).
- Enum verified against jazzy `nav2_msgs/CollisionMonitorState.msg`:
  **3 = APPROACH** (not LIMIT — LIMIT is 4). So the gate is confirmed
  **FootprintApproach APPROACH**, and **no `STOP` (1) appears ⇒ the
  invalid-source / TF-starvation stop is ruled out** for this stall.
- Toggling 3↔0 = publish-on-change: Nav2 commands motion → zeroed → command
  drops → retry. Classic wedge loop.
- **Still open — static check vs approach simulation** (the F29 go/no-go):
  APPROACH covers both the velocity-blind static check (≥ `min_points` 6 scan
  points inside the 0.17 m footprint ⇒ ALL cmd_vel zeroed incl. reverse) and the
  forward simulation (only motion toward points gated). Next measurements:
  - `ros2 topic echo /collision_monitor/collision_points_marker` (lazy topic) —
    count points within 0.17 m of base center.
  - Compare monitor input vs output cmd_vel `linear.x`: ratio 0.0 ⇒ full gate
    (static check likely); 0<r<1 ⇒ simulation throttle.
  - While wedged, publish a small negative-x cmd_vel into the monitor and watch
    the output — direct test whether reverse passes.
  - Decision: ≥6 points inside footprint ⇒ static check ⇒ F29 BackUp escape
    needs the `FootprintApproach.enabled false` dynamic toggle; <6 ⇒ plain
    BackUp viable.
- F29 has **no task file yet** (TF29 needed before code; probe should be T01).

Also this session — `nav2_params_explore_real_mini.yaml` review + wall-hug fix:

- **Header CHANGES table was stale** (claimed local inflation 0.15 and a global
  0.5 that was never in the file). Synced to the in-place annotations; rule-6
  stripped diff vs upstream now clean.
- **Robot drove too close to walls.** Local costmap `cost_scaling_factor`
  5.0 → 3.0 (back to upstream): higher scaling = faster cost decay = cheaper
  near-wall cells = MPPI hugs. If still too close next lever is local
  `inflation_radius` 0.4 → 0.55. Needs `colcon build` + real-robot retest.
- Literate docs synced to pending source changes: 04 (`Sequence[int]` uncopied
  map view), 06 (`world_to_cell` floor fix, inset-adjusted cluster scoring),
  08 (merge_tuning invariant, novelty telemetry keys); deleted
  `11-sim_explore_launch.md` (source launch file removed).

## Prior sessions (2026-07-18/20/21) — shipped + diagnosed

- **F27 lethal-goal guard shipped + verified live** (code/tests done; sim T06 /
  live T07 verification tasks still open). Robot no longer sends goals onto
  lethal cells. Fixed the `on_goal_result` stale-callback race (live `TypeError`
  crash) with regression tests.
- **Mini-config crash tuning** (`nav2_params_explore_real_mini.yaml`):
  `time_before_collision` 0.5→1.0, `robot_radius` 0.15→0.17 (true 0.16 + flared
  post base), MPPI+smoother linear speed 0.5→0.25. `STUCK_T_S` 7→20 so the
  explorer stops pre-empting Nav2 recovery.
- **Wedge diagnosis:** start-wedged robot never moves; Nav2's inner recovery is
  ClearLocalCostmap+retry (useless vs a real obstacle); Spin/BackUp live in the
  outer recovery it never reaches within the stuck window. → **F29** custom
  BackUp escape (feature written; collision_monitor source read 2026-07-21
  upgraded the probe to mandatory — reverse may be gated by the static check).
- **F28** (reason-tagged goal exclusion, per-reason TTL) and **F30**
  (path-distance Dijkstra frontier ranking, replaces Euclidean — kills
  through-wall goal picks) feature files written 2026-07-20/21. Neither has a
  task file yet.
- Pi is CPU-starved during nav: MPPI 8.6 Hz vs 20 desired; slam_toolbox TF
  queue-full drops. Throughput problem, upstream of any tolerance tuning.

## Status

Sim exploration works; robot drives and covers the map (~16 goals over ~9×9 m).
Full sim stack healthy. Real robot: explore runs but **start-wedged near an
obstacle it stalls** (this session's investigation). Modes A/B not live-verified.

**Dev VM has 1 core** — Nav2 is multi-process, so everything serializes:
intermittent action-ACK timeouts. Highest-impact fix = more vCPUs (4–6), not YAML.

Known-but-unfixed nav tuning:
- Planner choice unsettled: real configs SmacPlanner2D, sim NavFn.
- Real-robot MPPI CPU high; candidates `batch_size` 1000→500, freq 20→10 Hz.
- `FootprintApproach` `enabled: true` needs restoring in
  `nav2_params_explore_sim.yaml` + `nav2_params_explore_real.yaml` (disabled for
  diagnostics).

## Architecture essentials

- **One explorer node for sim and real:** `explorer_manager_node.py`
  (injected `ExplorationAlgorithm`, default `FrontierAlgorithm`). Sim vs real
  differ only by ROS params.
- **F23 decoupling:** node knows nothing about frontiers. Protocol =
  `next_goal(ctx) -> GoalDecision` (`NEW_GOAL/NO_TARGETS_BLOCKED/EXPLORED_DONE`);
  viz/diag/telemetry are optional opaque hooks via `getattr`. Frontier params
  self-declared by the algorithm (`frontier_params.FrontierParams`).
- **No YAML patching.** `config/` holds standalone commented copies of upstream
  defaults; launch files load them verbatim. Derived configs mark deltas with
  `# UPSTREAM <val>: why`.
- **slam** runs via upstream `online_async_launch.py`; maps persisted by
  `slam_manager_node` (`--map_name`). Re-running an existing name **overwrites**.
- **Gotcha — copy-install:** `colcon build --packages-select dome_nav` after
  every source edit.
- **Gotcha — orphan processes:** stale nodes/`gz sim` cause TF/clock collisions;
  `ps` audit + `kill -9` beats `pkill -f`.

## Key params (real default / sim override)

Frontier params owned + self-declared by `FrontierAlgorithm`; node declares only
the shared set.

- `min_frontier_dist`: 0.5 / **0.9** m; `max_frontier_dist`: 0.0 / **15.0** m
- `min_frontier_size`: 15 default / **5** sim / 10 real (launch)
- `frontier_buffer_cells`: 2; `goal_inset_m`: 0.3
- `preferred_goal_distance`: 1.0 real / 2.0 sim — `min |d - preferred|`
- `use_novelty_scoring`: False (F15, opt-in); `novelty_top_n`: 5
- `max_explore_radius`: 0.0; `blacklist_radius`: 0.5 m
- Node constants: `EXPLORE_HZ` 1, `NO_FRONTIER_PATIENCE` 14,
  `GOAL_TIMEOUT_S` 25, `STUCK_T_S` 20, `MAX_GOAL_ATTEMPTS` 8,
  `LETHAL_THRESHOLD` 99 (scaled OccupancyGrid — one scale, node + diagnostics)

## Launch

```bash
# Real robot — base stack first (no nav), then a mode:
bl dome2 robot.launch.py --options "drivers control vision voice"
bl dome_nav robot_map.launch.py --map_name <name>      # Mode A: mapping (slam)
bl dome_nav robot_nav.launch.py                        # Mode B: AMCL nav
bl dome_nav robot_explore.launch.py --map_name <name>  # Mode E: autonomous explore

# Sim — single command:
bl dome_nav sim_nav_full.launch.py --map_name <name> --world_name multi_room
# sim_rviz.launch.py separate optional window.

# Experiment harness (trimmed nav2, C2 CPU fix):
bl dome_nav nav_experiment.launch.py
```

`just_explorer.launch.py` is new + untracked (explorer node alone).

## Exploration control

```bash
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_start\", \"source\": \"cli\", \"slots\": {}}"'
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_stop\",  \"source\": \"cli\", \"slots\": {}}"'
ros2 topic echo /explore/status
tail -f ~/.dome/telemetry/e*.json       # e<mapname><dd-mmm>.json (F17)
```

Intent contract: `nav go <label>`→`navigation_go {label}`, `nav cancel`→
`navigation_cancel`, `nav explore`→`exploration_start`, `nav explore stop`→
`exploration_stop`. `/explore/markers`: frontiers yellow, blacklist red, goal cyan.

## Collision monitor probe commands (this investigation)

```bash
ros2 topic echo /collision_monitor_state                      # action change; 3=APPROACH, 1=STOP
ros2 topic echo /collision_monitor/collision_points_marker    # base-frame points (lazy)
# throttle = out/in cmd_vel linear.x ratio
ros2 param set /collision_monitor FootprintApproach.enabled false   # dynamic escape toggle
```

## Next steps

1. **Finish gate probe** (points-in-footprint count + cmd_vel ratio + reverse
   test) → decides F29 design. Then write TF29 with probe as T01.
2. **Write TF30** (path-distance ranking, High) — biggest stall-input fix.
3. **Give the dev VM 4–6 vCPUs.**
4. **Restore `FootprintApproach` enabled** in both explore configs.
5. TF27 T06 sim verify (lethal-goal guard skip log).
6. TF15 T05 live verify (novelty on vs off).
7. Real-robot retest of wall standoff (local `cost_scaling_factor` 5.0→3.0).

## In-flight features

- **F27** lethal-goal guard: code+tests done, live-observed; T06 sim + T07 live
  verification pending — feature open.
- **F29** BackUp escape: feature file only; probe in progress (this session);
  no TF29 yet.
- **F30** path-distance ranking: feature file only; no TF30 yet.
- **F28** reason-tagged exclusion: feature file only; no TF28 yet.
- **F26** survey-algorithms paper: TF26 T01–T05 not started.
- **F15** novelty scoring: code done; T05 live verify + literate regen pending.
- **F10** exploration: T06 hardware questions + T07 live smoke pending.
- **F09** dome_control integration: T04 live smoke pending.
- **F05** rosbag integration test: not started, no task file.

## Open issues

`05-issues/open/` is empty.
