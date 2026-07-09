# dome_nav — Current Session Handoff

Concise cold-start orientation. Detailed history lives in git log and the
`04-tasks/` files — do **not** re-narrate it here.

**Date:** 2026-07-09 · **Branch:** main

## Status

Sim exploration works and the robot **drives and covers the map** (observed ~16
goals over a ~9×9 m area in one run). Full sim stack (Gazebo + slam_toolbox +
Nav2 + explore) comes up healthy. Real-robot Modes A/B/E have **not** been
live-run — treat them as unverified.

**Performance ceiling — the dev VM has only 1 core** (`nproc` = 1, on an M2 Mac).
Nav2 is multi-process and MPPI parallelizes across cores, so a single core (however
fast) serializes everything: MPPI/NavFn solves block the action-server ACK
callbacks → intermittent `Timed out while waiting for action server to acknowledge`
aborts. **Highest-impact fix is to give the VM more vCPUs (→4–6), not more YAML.**

Recent changes (2026-07-09, this session):
- **Frontier buffer 1→2 cells**, parameterized as `frontier_buffer_cells` (ROS param,
  default 2). `find_frontier_clusters` now walks N known-cell rings inward from the
  unknown boundary. Guards goals against the SLAM-map vs. global-costmap seam.
- **Costmap-bounds goal reject**: `explore_tick` now skips any candidate goal that maps
  outside `/global_costmap` (`goal_in_global_costmap`) and re-asks for the next-best
  frontier, up to `MAX_GOAL_ATTEMPTS` (8). Fixes `worldToMap`→`PLAN/NO_VALID_PATH`.
- **MPPI/motion fixes in `nav2_explore_sim.yaml`** — diagnosed "robot receives paths
  for 25 s but moves 3 cm": (a) `velocity_smoother.deadband_velocity` reset to
  `[0,0,0]` (the 0.05/0.1 deadband zeroed the small hesitant commands a CPU-starved
  MPPI emits — froze the robot in sim; kept in real config); (b) `batch_size 2000→1000`;
  (c) `visualize true→false`. (d) `FootprintApproach` collision_monitor `enabled: false`
  (diagnostic — restore when done).
- New experimental `launch/sim_nav_default.launch.py` (loads **stock** nav2_params to
  bisect config vs. sim wiring — proved the sim/robot wiring is fine). Delete when done.
- `expresume.bash` helper (publishes `exploration_resume`).

Earlier 2026-07-09 changes: `prefer_farthest=True` (real), sequential telemetry
(`exp-0001.json`), `dump_failure_diagnostics`/`dump_frontier_exhaustion`,
`paused_on_failure` + `exploration_resume`, costmap subscriptions, `NAV2_ERROR_CODES`.

Known-but-unfixed nav tuning issues (none block basic exploration):
- Intermittent action-ACK timeouts under load — **root cause is the 1-core VM** (above).
- Planner choice unsettled: explore configs use NavFn, `nav2_real.yaml` uses SmacPlanner2D.
  Candidate: switch sim to `SmacPlanner2D` + `use_astar` (faster replan, fewer ACK timeouts).
- `prefer_farthest=True` in sim aims at the farthest frontier (most likely across
  unmapped space); `False` (nearest-first) may explore more robustly.

## Architecture essentials

- **One explorer node for sim and real:** `pluggable_explore_manager_node.py`
  (injected `ExplorationAlgorithm`, default `FrontierAlgorithm`). The old
  `explore_manager_node.py` was deleted. Sim vs real differ only by ROS params.
- **No YAML patching.** `config/` holds six standalone, commented copies of the
  upstream defaults: `slam_real.yaml`, `slam_sim.yaml`, `nav2_real.yaml`
  (Modes A/B nav), `nav2_localization_real.yaml` (Mode B AMCL),
  `nav2_explore_real.yaml`, `nav2_explore_sim.yaml`. Launch files load these
  verbatim via the standard `bl.include(...)`. `utils.py` config helpers are down
  to `write_config` (+ `dome_home`/world helpers).
- **slam** runs via standard `online_async_launch.py`. No `map_file_name` — maps
  are persisted by `slam_manager_node` (`map_persist_path` = `--map_name`). Note:
  re-running an existing `--map_name` **overwrites** rather than resumes.
- **Gotcha — copy-install:** run `colcon build --packages-select dome_nav` after
  every source edit before `bl`/`ros2 run` sees it.
- **Gotcha — orphan processes:** stale nodes/`gz sim` across runs cause TF/clock
  collisions. `ps` audit + explicit `kill -9` beats trusting `pkill -f`.

## Key params (node ROS params; real default / sim override)

- `min_frontier_dist`: 0.5 / **0.9** m (raw frontier-cell floor; `goal_inset` 0.3 pulls the sent goal 0.3 m closer)
- `max_frontier_dist`: 0.0 (unlimited) / **15.0** m
- `min_frontier_size`: 10 / **5** cells
- `prefer_farthest`: **True** (real and sim)
- `frontier_buffer_cells`: 2 (known-cell rings between a frontier goal and unknown)
- `max_explore_radius`: 0.0 (unlimited); `goal_inset_m`: 0.3; `blacklist_radius`: 0.5 m
- Constants: `EXPLORE_HZ` 2, `NO_FRONTIER_PATIENCE` 14 ticks (must exceed slam's 5 s `map_update_interval`), `GOAL_TIMEOUT_S` 25 s, `MAX_GOAL_ATTEMPTS` 8
- Sim goal checker: `yaw_goal_tolerance` ~π (goals sent with identity orientation; exploration doesn't care about final heading)

## Launch

```bash
# Real robot — base stack first (no nav), then a mode:
bl dome2 robot.launch.py --options "drivers control vision voice"
bl dome_nav robot_map.launch.py --map_name <name>      # Mode A: mapping (slam)
bl dome_nav robot_nav.launch.py                        # Mode B: AMCL nav (uses saved basement1 map)
bl dome_nav robot_explore.launch.py --map_name <name>  # Mode E: autonomous explore

# Sim — two steps (Gazebo started separately, then the stack):
gz sim -r ~/ros2_ws/install/dome_nav/share/dome_nav/worlds/multi_room.world
bl dome_nav sim_nav_full.launch.py --map_name <name> --world_name multi_room
# sim_rviz.launch.py is a separate optional window.
```

## Exploration control

```bash
# start / stop (dome_control sends these; or by hand):
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_start\", \"source\": \"cli\", \"slots\": {}}"'
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_stop\",  \"source\": \"cli\", \"slots\": {}}"'
ros2 topic echo /explore/status          # {"state","reached","failed",... goal_xy,dist_m,elapsed_s}
tail -f ~/.dome/telemetry/exp-*.json    # session_start/goal_sent/goal_result/no_frontier/session_end
```

Intent contract: `nav go <label>`→`navigation_go {label}`, `nav cancel`→`navigation_cancel`,
`nav explore`→`exploration_start`, `nav explore stop`→`exploration_stop`.
`/explore/markers` (MarkerArray): frontiers (yellow), blacklist (red), goal (cyan).

## Next steps

1. **Give the dev VM 4–6 vCPUs** (currently 1) — the single biggest reliability win;
   should clear most intermittent action-ACK timeout aborts. Then re-run and confirm
   MPPI control-loop rate recovers toward 20 Hz.
2. **Restore `FootprintApproach` `enabled: true`** in `nav2_explore_sim.yaml` once the
   no-motion investigation is fully closed (currently disabled for diagnostics).
3. **Delete `launch/sim_nav_default.launch.py`** (experimental bisect) when done.
4. **F13 T05/T06** — call the sim exploration demo done; move F13 feature/task to done.
5. **Optional Nav2 tuning** — SmacPlanner2D+`use_astar` for sim; revisit `prefer_farthest`.
6. **Real-robot verification (F10 T07)** — Modes A/B/E have never run on hardware.

## Open issues

None. `05-issues/open/` is empty (I01–I11 all closed).
