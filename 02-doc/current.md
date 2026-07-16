# dome_nav — Current Session Handoff

Concise cold-start orientation. Detailed history lives in git log and the
`04-tasks/` files — do **not** re-narrate it here.

**Date:** 2026-07-16 · **Branch:** main

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

Recent changes (2026-07-16, this session — docs/refactor scaffolding, no nav behavior change):
- **Experiment logs consolidated:** `experiment.md` + `experiment_navfail.md` →
  single `experiments.md` (deadband bug, start-in-inflation deadlock, Pi CPU
  campaign, fail-fast reselection). E7 stub added; E5/E6/E7 pending hardware runs.
- **Node renamed:** `pluggable_explore_manager_node.py` → `explorer_manager_node.py`,
  class `PluggableExploreManagerNode` → `ExplorerManagerNode` (ROS graph name
  `explore_manager_node` kept stable). Entry point, launch files, test, literate 09
  all updated.
- **F22 (hello-world plugin):** T01 architecture critique DONE, T02
  `HelloWorldAlgorithm` DONE (`dome_nav/hello_world_algorithm.py`, literate X08).
  T03 (runtime `explore_algorithm` selector) / T04 (tests) / T05 (demo) pending.
- **F23 (decouple manager from frontier):** new feature + TF23 tasks, converted
  from the T01 critique. Removes 3 leaks — lossy `None` return + `latest_clusters`
  side-channel, node-hardcoded frontier done-rule, frontier-heavy `ExploreParams`.
  Issue I12 closed → F23 (retained as backing critique).

Earlier changes (2026-07-10):
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
  caused premature "done". `controller_server` 70% CPU from MPPI (expected on Pi).
  `nav2_params_explore_real.yaml` already runs `batch_size 1000` (halved from 2000 on
  2026-07-09); remaining candidate fix: lower `controller_frequency` 20→10 Hz (and,
  if still CPU-bound, `batch_size` 1000→500) for the real-robot explore config.

Earlier 2026-07-09 changes: frontier buffer 1→2 cells, costmap-bounds goal reject,
`prefer_farthest=True` (real), sequential telemetry, `dump_failure_diagnostics`,
`paused_on_failure` + `exploration_resume`, MPPI/motion fixes.

Known-but-unfixed nav tuning issues (none block basic exploration):
- Intermittent action-ACK timeouts under load — **root cause is the 1-core VM** (above).
- Planner choice unsettled: `nav2_params_explore_real.yaml` and `nav2_params_real.yaml` use
  SmacPlanner2D; `nav2_params_explore_sim.yaml` uses NavFn.
- `prefer_farthest=True` in sim; F14 will replace this with `preferred_goal_distance`.
- Real-robot MPPI CPU load high (70%); candidate: `batch_size` 2000→500, freq 20→10 Hz.

## Architecture essentials

- **One explorer node for sim and real:** `explorer_manager_node.py`
  (injected `ExplorationAlgorithm`, default `FrontierAlgorithm`). The old
  `explore_manager_node.py` was deleted. Sim vs real differ only by ROS params.
- **No YAML patching.** `config/` holds six standalone, commented copies of the
  upstream defaults: `mapper_params_online_async.yaml`, `mapper_params_online_async_sim.yaml`, `nav2_params_real.yaml`
  (Modes A/B nav), `nav2_params_localization_real.yaml` (Mode B AMCL),
  `nav2_params_explore_real.yaml`, `nav2_params_explore_sim.yaml`. Launch files load these
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
- `preferred_goal_distance`: **1.0 m** (real) / **2.0 m** (sim) — selects frontier cell with `min |d - preferred_dist|`; `prefer_farthest` deprecated
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

# Sim — single command (Gazebo launched inside sim_robot.launch.py):
bl dome_nav sim_nav_full.launch.py --map_name <name> --world_name multi_room
# sim_rviz.launch.py is a separate optional window.
```

## Exploration control

```bash
# start / stop (dome_control sends these; or by hand):
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_start\", \"source\": \"cli\", \"slots\": {}}"'
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_stop\",  \"source\": \"cli\", \"slots\": {}}"'
ros2 topic echo /explore/status          # {"state","reached","failed",... goal_xy,dist_m,elapsed_s}
tail -f ~/.dome/telemetry/e*.json       # e<mapname><dd-mmm>.json (F17); old exp-NNNN.json also present
```

Intent contract: `nav go <label>`→`navigation_go {label}`, `nav cancel`→`navigation_cancel`,
`nav explore`→`exploration_start`, `nav explore stop`→`exploration_stop`.
`/explore/markers` (MarkerArray): frontiers (yellow), blacklist (red), goal (cyan).

## Next steps

1. **Give the dev VM 4–6 vCPUs** (currently 1) — single biggest reliability win.
2. **Restore `FootprintApproach` `enabled: true`** in both `nav2_params_explore_sim.yaml`
   and `nav2_params_explore_real.yaml` (currently disabled for diagnostics in both).
3. **Delete `launch/sim_nav_default.launch.py`** (experimental bisect artifact).
4. **F17 done** — telemetry files now named `e<map_name><dd-mmm>.json`; old `exp-NNNN.json` files coexist untouched. dome_control CSV rename (`t<dd-mmm>.csv`) still pending in dome_control.
5. **F14 done** — `preferred_goal_distance` param replaces `prefer_farthest`; selection is `min |d - preferred_dist|`. `prefer_farthest` kept as deprecated alias (logs warning, maps True→max_frontier_dist). Sim default 2.0 m, real 1.0 m.
6. **F16 done** — `save_period_sec` default 60→120 s; `export_legacy_map: bool = True` param; `/slam_toolbox/save_map` called after each posegraph save (best-effort).
7. **F15** — implement path novelty scoring (opt-in, after F14 landed).
8. **Reduce MPPI CPU on real robot** — try `batch_size` 2000→500, `controller_frequency`
   20→10 Hz in `nav2_params_explore_real.yaml`.
9. **Real-robot verification (F10 T07)** — Modes A/B/E never run on hardware.
10. **F23 T01 (next focused session)** — intent-carrying `next_goal` result
    (`NEW_GOAL/NO_TARGETS_BLOCKED/EXPLORED_DONE`); the keystone that removes the
    `latest_clusters` peek and moves the done-decision into the algorithm. Touches
    protocol + both algorithms + node + tests together.
11. **F22 T03/T04** — runtime `explore_algorithm` selector + unit tests, to finish
    the hello-world plugin end-to-end.

## In-flight features

- **F22** hello-world plugin: T01–T02 done; T03–T05 pending.
- **F23** decouple manager from frontier: tasks TF23 defined; not started.

## Open issues

`05-issues/open/` is empty. I12 (interface leak) closed → converted to F23.
