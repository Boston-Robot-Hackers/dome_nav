# dome_nav — Current Session Handoff

Concise cold-start orientation. Detailed history lives in git log and the
`04-tasks/` files — do **not** re-narrate it here.

**Date:** 2026-07-10 · **Branch:** main

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

Recent changes (2026-07-10, this session):
- **Feature files F14–F17 added** (no code changes):
  - F14: `preferred_goal_distance` param replaces binary `prefer_farthest`; ranks
    frontiers by `|d - preferred_dist|` instead of nearest/farthest.
  - F15: path novelty scoring — Bresenham unknown-cell count on straight line to each
    candidate; opt-in via `use_novelty_scoring` param.
  - F16: periodic map save every 2 min (change default) + legacy PNG/YAML export via
    `/slam_toolbox/save_map` service after each save.
  - F17: telemetry filename rename — `e<map_name><dd-mmm>.json` replaces `exp-NNNN.json`;
    dome_control CSV rename (`t<dd-mmm>.csv`) also documented here (change lives in dome_control).
- **Real-robot telemetry analysis (`exp-0004.json`)**: identified that y≈0.7 corridor
  is physically blocked on the real map; blacklist over-accumulation (radius 0.5 m)
  caused premature "done". `controller_server` 70% CPU = MPPI `batch_size 2000`
  (expected on Pi). Candidate fix: lower `batch_size` to 500 and `controller_frequency`
  to 10 Hz for real-robot explore config.

Earlier 2026-07-09 changes: frontier buffer 1→2 cells, costmap-bounds goal reject,
`prefer_farthest=True` (real), sequential telemetry, `dump_failure_diagnostics`,
`paused_on_failure` + `exploration_resume`, MPPI/motion fixes.

Known-but-unfixed nav tuning issues (none block basic exploration):
- Intermittent action-ACK timeouts under load — **root cause is the 1-core VM** (above).
- Planner choice unsettled: explore configs use NavFn, `nav2_real.yaml` uses SmacPlanner2D.
- `prefer_farthest=True` in sim; F14 will replace this with `preferred_goal_distance`.
- Real-robot MPPI CPU load high (70%); candidate: `batch_size` 2000→500, freq 20→10 Hz.

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
tail -f ~/.dome/telemetry/exp-*.json    # current naming; F17 will change to e<mapname><dd-mmm>.json
```

Intent contract: `nav go <label>`→`navigation_go {label}`, `nav cancel`→`navigation_cancel`,
`nav explore`→`exploration_start`, `nav explore stop`→`exploration_stop`.
`/explore/markers` (MarkerArray): frontiers (yellow), blacklist (red), goal (cyan).

## Next steps

1. **Give the dev VM 4–6 vCPUs** (currently 1) — single biggest reliability win.
2. **Restore `FootprintApproach` `enabled: true`** in `nav2_explore_sim.yaml` (currently
   disabled for diagnostics).
3. **Delete `launch/sim_nav_default.launch.py`** (experimental bisect artifact).
4. **F13 T05/T06** — run end-to-end sim smoke test; declare F13 done.
5. **F17** — implement telemetry filename rename in `explore_telemetry.py`; coordinate
   dome_control CSV rename separately.
6. **F14** — implement `preferred_goal_distance`; deprecate `prefer_farthest`.
7. **F16** — implement periodic map save default + legacy PNG/YAML export.
8. **F15** — implement path novelty scoring (opt-in, after F14 landed).
9. **Reduce MPPI CPU on real robot** — try `batch_size` 2000→500, `controller_frequency`
   20→10 Hz in `nav2_explore_real.yaml`.
10. **Real-robot verification (F10 T07)** — Modes A/B/E never run on hardware.

## Open issues

None. `05-issues/open/` is empty (I01–I11 all closed).
