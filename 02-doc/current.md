# dome_nav — Current Session Handoff

Concise cold-start orientation. Detailed history lives in git log and the
`04-tasks/` files — do **not** re-narrate it here.

**Date:** 2026-07-17 · **Branch:** main

## This session (2026-07-17)

- **F15 path novelty scoring — landed (code+tests), live-verify (TF15 T05) pending.**
  Pure `path_novelty_score`/`best_frontier_candidates`/`pick_by_novelty` in
  `frontier_explorer.py`; opt-in `use_novelty_scoring`/`novelty_top_n` in
  `FrontierParams`; `FrontierAlgorithm.select_target` branch; novelty rides the opaque
  `telemetry_extra` (`novelty_score`) — node untouched (F23 intact). Off by default.
- **F24 remove periodic map save — landed.** Stripped the `save_period_sec` timer +
  `periodic_save` from `slam_manager_node.py` (reverses F16's periodic save only; keeps
  first-map + shutdown saves, both still modern+legacy). Dropped `save_period_sec`
  overrides in `sim_nav_full`/`sim_explore` launches. **`kill -9` now keeps only the
  first-map save** (no periodic fallback).
- **F25 minimal real explore config — file added.** `config/nav2_params_explore_real_mini.yaml`
  = upstream `nav2_params.yaml` + 3 surgical deltas: `robot_radius` 0.22→0.15 (×2),
  `time_before_collision` 1.2→0.5 (E6/E7, **UNVERIFIED**), deadband kept [0,0,0]. Not
  wired into any launch; opt-in via `--nav2_config`.
- **Deleted `experiments.md` + `experiments/` (5 yamls).** Recoverable via git history
  only. Bug 2 root cause + Pi CPU campaign (C1/C2/C4) findings now live ONLY in git
  history — not migrated to notes.md yet.
- Literate refreshed: `02-slam_manager_node` (v3.4), `06-frontier_explorer` (v1.7),
  `08-frontier_algorithm` (v1.1). Full suite: **224 passed, 4 deselected**.


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

Recent changes (2026-07-16, this session — **F23 T01–T03 decoupling landed**, no nav behavior change):
- **F23 T01 — intent-carrying result:** `next_goal(ctx) -> GoalDecision`
  (`NEW_GOAL(xy)/NO_TARGETS_BLOCKED/EXPLORED_DONE`, in `explore_context.py`).
  Algorithm owns the done-condition; node dropped its `latest_clusters` done-peek.
  `EXPLORED_DONE` ends the session immediately (hello no longer waits out patience).
- **F23 T02 — viz/diag off the protocol:** protocol requires only `next_goal`.
  Markers/exhaustion/failure/telemetry are OPTIONAL opaque hooks the node calls via
  `getattr` (`render_markers`/`exhaustion_report`/`failure_report`/`telemetry_extra`/
  `session_params`); new `RenderContext` carries node-general session state only.
  Node reads **zero** algorithm internals; `latest_clusters`/`latest_diag` are now
  private to `FrontierAlgorithm`.
- **F23 T03 — split params:** new `frontier_params.FrontierParams` owns frontier
  tuning; `ExploreParams` keeps only the shared set. Algorithm self-declares its ROS
  params via `declare_params(node)`; the node declares **no** frontier param names.
- **Node has no frontier params/tuning left** (session telemetry routes through the
  opaque `session_params` hook). Remaining `frontier`/`cluster` hits in the node are
  naming only (`no_frontier_count`, `NO_FRONTIER_PATIENCE`) + the registry default —
  T04 audit / rename chore. `STUCK_T_S`/`GOAL_TIMEOUT_S` are navigation, node-owned.
- Earlier this session: node renamed
  `pluggable_explore_manager_node.py` → `explorer_manager_node.py`.
  (`experiments.md` + `experiments/` were later deleted — see git history for the
  Bug 1/Bug 2/CPU investigation log.)
- **F22 (hello-world plugin):** T01–T02 done; hello updated for the F23 protocol
  (declare_params no-op, no faked cluster state). T03–T05 pending.

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

## Key params (ROS params; real default / sim override)

**Since F23 T03 the frontier params are owned + self-declared by `FrontierAlgorithm`**
(`frontier_params.FrontierParams` / `declare_frontier_params`), not the node. Same
ROS names, still yaml/launch-settable; the node declares only the shared set.

- `min_frontier_dist`: 0.5 / **0.9** m (raw frontier-cell floor; `goal_inset` 0.3 pulls the sent goal 0.3 m closer) — *frontier*
- `max_frontier_dist`: 0.0 (unlimited) / **15.0** m — *frontier*
- `min_frontier_size`: 15 default / **5** sim / launch overrides to 10 real — *frontier*
- `frontier_buffer_cells`: 2 (known-cell rings between a frontier goal and unknown) — *frontier*
- `goal_inset_m`: 0.3 — *frontier*
- `preferred_goal_distance`: **1.0 m** (real) / **2.0 m** (sim) — *shared*; selects frontier cell with `min |d - preferred_dist|`; `prefer_farthest` deprecated (frontier)
- `max_explore_radius`: 0.0 (unlimited); `blacklist_radius`: 0.5 m — *shared*
- Node constants (navigation/session, not frontier): `EXPLORE_HZ` 1, `NO_FRONTIER_PATIENCE` 14 ticks (must exceed slam's 5 s `map_update_interval`), `GOAL_TIMEOUT_S` 25 s, `STUCK_T_S` 7 s, `MAX_GOAL_ATTEMPTS` 8
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
10. *(none — F22 and F23 are closed)*

## In-flight features

- **F22** hello-world plugin: **T01–T05 done**, feature closed.
- **F23** decouple manager from frontier: **T01–T05 done**, feature closed.

## Open issues

`05-issues/open/` is empty. I12 (interface leak) closed → converted to F23.
