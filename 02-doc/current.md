# dome_nav — Current Session Handoff

Concise cold-start orientation. Detailed history lives in git log and the
`04-tasks/` files — do **not** re-narrate it here.

**Date:** 2026-07-08 · **Branch:** main (pushed through `24833e9`)

## Status

Sim exploration works: goals are sent and reached, the map fills in. Full sim
stack (Gazebo + slam_toolbox + Nav2 + explore) comes up healthy. Real-robot
Modes A/B/E have **not** been live-run — treat them as unverified.

Known-but-unfixed nav tuning issues (candidates for a Nav2 discussion, none
block basic exploration):
- ~0.11 m/s crawl on short (~1 m) goals — stock MPPI `GoalCritic.threshold_to_consider: 1.4`.
- Near-wall stalls — planner "Start occupied" when the robot center sits in an inflated/lethal cell.
- Planner choice unsettled: explore configs use NavFn, `nav2_real.yaml` uses SmacPlanner2D.

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

- `min_frontier_dist`: 1.3 / **0.9** m (raw frontier-cell floor; `goal_inset` 0.3 pulls the sent goal 0.3 m closer)
- `max_frontier_dist`: 0.0 (unlimited) / **15.0** m
- `min_frontier_size`: 10 / **5** cells
- `prefer_farthest`: False / **True**
- `max_explore_radius`: 0.0 (unlimited); `goal_inset_m`: 0.3; `blacklist_radius`: 0.5 m
- Constants: `EXPLORE_HZ` 2, `NO_FRONTIER_PATIENCE` 14 ticks (must exceed slam's 5 s `map_update_interval`), `GOAL_TIMEOUT_S` 25 s
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
tail -f ~/.dome/telemetry/*.jsonl        # session_start/goal_sent/goal_result/no_frontier/session_end
```

Intent contract: `nav go <label>`→`navigation_go {label}`, `nav cancel`→`navigation_cancel`,
`nav explore`→`exploration_start`, `nav explore stop`→`exploration_stop`.
`/explore/markers` (MarkerArray): frontiers (yellow), blacklist (red), goal (cyan).

## Next steps

1. **F13 T05/T06** — call the sim exploration demo done; move F13 feature/task to done.
2. **Nav2 tuning pass** (optional) — the three issues under Status above.
3. **Real-robot verification (F10 T07)** — Modes A/B/E have never run on hardware;
   `nav2_real.yaml`/`nav2_localization_real.yaml` are behavior-preserving copies of the
   old merges but unproven.

## Open issues

None. `05-issues/open/` is empty (I01–I11 all closed).
