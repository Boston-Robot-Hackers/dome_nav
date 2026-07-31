# Tunable Parameters Guide

This document inventories every parameter you can use to tune the exploration and
navigation behavior of `dome_nav`, grouped by where it lives:

0. [Most useful tunings (start here)](#0-most-useful-tunings-start-here)
1. [ROS2 parameters — explorer manager (shared)](#1-ros2-parameters-explorer-manager-shared)
2. [ROS2 parameters — frontier algorithm](#2-ros2-parameters-frontier-algorithm)
3. [ROS2 parameters — slam manager](#3-ros2-parameters-slam-manager)
4. [Launch-file exposure](#4-launch-file-exposure)
5. [YAML configs (slam_toolbox / nav2)](#5-yaml-configs)
6. [Hard-coded constants (code-edit only)](#6-hard-coded-constants)
7. [Architecture notes](#7-architecture-notes)
8. [Launch-file inventory & redundancy](#8-launch-file-inventory--redundancy)

**Runtime-configurable** means the value can be set in a launch file or YAML and changed
without editing code. **Code-edit only** means the default is baked into the source.

---

## 0. Most useful tunings (start here)

If you only touch a handful of knobs, touch these. All are runtime-configurable
(launch-file or YAML) and have the biggest, most predictable effect on behavior.

Recommended-value columns are the values the shipped launch/YAML configs actually
set (sim = `sim_explore_node.launch.py` / `nav2_params_explore_sim.yaml`;
real = `robot_explore.launch.py` / `nav2_params_explore_real.yaml`). Use them as
the starting point, not the dataclass defaults.

### Exploration coverage vs. speed

| Parameter | Section | Sim | Real | Effect | Tune up when | Tune down when |
|---|---|---|---|---|---|---|
| `preferred_goal_distance` | [2](#2-ros2-parameters-frontier-algorithm) | `2.0` | `2.0` | How far the robot travels per goal. Biggest lever on explore pace and thrashing. | Robot dithers among nearby frontiers; want longer purposeful runs. | Robot overshoots into unmapped space and gets stuck. |
| `min_frontier_dist` | [2](#2-ros2-parameters-frontier-algorithm) | `0.9` | `0.5` | Rejects goals too close to the robot. Kills oscillation near the current pose. | Robot picks goals right on top of itself. | Robot ignores nearby unmapped pockets. |
| `max_frontier_dist` | [2](#2-ros2-parameters-frontier-algorithm) | `15.0` | `0.0` (off) | Upper bound on robot-to-goal distance. Sim caps it; real leaves it open. | Robot commits to far goals across unmapped space. | Want only-nearby goals. |
| `max_explore_radius` | [1](#1-ros2-parameters-explorer-manager-shared) | `0.0` | `0.0` | Hard cap on how far from start to explore. | Never — keep `0.0` for full coverage. | Want to bound the session to one room/area. |
| `min_frontier_size` | [2](#2-ros2-parameters-frontier-algorithm) | `5` | `10` | Ignores small frontier clusters (noise/thin gaps). Real is stricter — noisier lidar. | Robot chases sensor-noise frontiers and never finishes. | Missing real but small openings (doorways). |

### Safety / clearance

| Parameter | Section | Sim | Real | Effect | Tune up when | Tune down when |
|---|---|---|---|---|---|---|
| `robot_radius` + `clearance_margin_m` | [2](#2-ros2-parameters-frontier-algorithm) | `0.17` + `0.05` | `0.17` + `0.05` | Clearance floor = `robot_radius + clearance_margin_m`; rejects goals too near walls. | Robot picks goals that wedge it against walls (see [[project_smac_collision_monitor]]). | Robot refuses valid goals in tight spaces. |
| costmap `inflation_radius` | [5](#5-yaml-configs) | `0.7` (local) | `0.25` local / `0.2` global | Buffer nav2 keeps from obstacles. Dominant nav-side safety/pass-ability knob. Sim runs fat; real trimmed to pass tight halls. | Robot clips corners / walls. | Robot treats narrow halls as impassable. |
| `lethal_cost_threshold` | [5](#5-yaml-configs) | `65` | `65` | /map cell value nav2 treats as wall. | False obstacles from noisy map. | Robot drives through soft obstacles. |

### Getting-unstuck (nav side)

| Parameter | Section | Sim | Real | Effect |
|---|---|---|---|---|
| goal_checker `xy_goal_tolerance` / `yaw_goal_tolerance` | [5](#5-yaml-configs) | `0.5` / `3.15` | (default) / `1.0` | How precisely nav2 must hit the goal. Loose = fewer "can't reach goal" hangs during exploration. Sim yaw wide-open (3.15 ≈ any heading). |
| MPPI `vx_max` / `wz_max` | [5](#5-yaml-configs) | `±0.45` / `az 3.2` | `±0.6` / `1.4` | Top speeds. Lower for tight/cluttered spaces, higher for open runs. |
| `GOAL_TIMEOUT_S` / `STUCK_T_S` | [6](#6-hard-coded-constants) | `25.0` / `20.0` | `25.0` / `20.0` | Watchdog timeouts that break nav2 recovery loops (code-edit only, same both). See [[project_nav2_stuck_investigation]]. |

**Rule of thumb**: tune exploration knobs (§1–2) first to change *what* the robot
targets; tune nav2 YAML (§5) to change *how* it drives there. Don't chase a
navigation symptom with an exploration knob or vice-versa.

---

## 1. ROS2 parameters — explorer manager (shared)

Declared on the node from the `ExploreParams` dataclass via
`declare_dataclass_params()` (`dome_nav/explore_context.py`); owned by the
node, shared by all algorithms. **Ownership rule (F34 T03): a field is shared
iff the node itself reads it** — radius gating (`max_explore_radius`) or
blacklist reselection (`blacklist_radius`). Tuning only an algorithm's scorer
reads (e.g. `preferred_goal_distance`) lives in that algorithm (§2), not here.
`explore_algorithm` and `map_name` are hand-declared on the node.

| Parameter | Default | Type | What it tunes |
|---|---|---|---|
| `explore_algorithm` | `"frontier"` | str | Algorithm selection from `ALGORITHM_REGISTRY` (`frontier` / `hello`, explorer_manager_node.py:47-51). Unknown name → warning + fallback. |
| `max_explore_radius` | `0.0` (unlimited) | float | Radius from session start pose beyond which frontier clusters are filtered out (frontier_explorer.py:303-308). |
| `blacklist_radius` | `0.5` | float | Exclusion radius around failed goals; node reselection suppresses this neighborhood. Kept `> goal_inset_m` by `merge_tuning` (frontier_params.py). |
| `map_name` | `"unknown"` | str | Telemetry filename tag only (`~/.dome/telemetry/e<map><date>.json`); no effect on mapping. |

---

## 2. ROS2 parameters — frontier algorithm

Defaults live in the `FrontierParams` dataclass (`dome_nav/frontier_params.py`);
each field carries its description/range as dataclass metadata and is declared
via the `fields()`-driven `declare_frontier_params()` when the algorithm
registers itself. All runtime-configurable.

### Goal / frontier filtering

| Parameter | Default | Type | What it tunes |
|---|---|---|---|
| `min_frontier_size` | `15` | int | Drop frontier clusters smaller than this many cells (frontier_explorer.py:299-300). |
| `min_frontier_dist` | `1.3` | float | Goal must be at least this far from the robot (0 disables). |
| `max_frontier_dist` | `0.0` (unlimited) | float | Upper bound on robot-to-goal distance (0 disables). |
| `preferred_goal_distance` | `1.0` | float | Target travel distance per goal; scoring minimizes \|actual reach − preferred\| (frontier_explorer.py:269-273). Algorithm-owned (F34 T03); `HelloWorldAlgorithm` declares its own same-named step param. |
| `goal_inset_m` | `0.3` | float | Pull the frontier goal this far toward the robot so it stays inside the costmap (frontier_explorer.py:460-474). ROS-declared but set by no launch file in the repo. |
| `frontier_buffer_cells` | `2` | int | How many free-cell rings inside the unknown boundary count as frontier during detection (frontier_explorer.py:23-89). |

### Scoring

| Parameter | Default | Type | What it tunes |
|---|---|---|---|
| `use_novelty_scoring` | `False` | bool | Opt-in: adds novelty scorer (unknown cells crossed en route) to the scoring pipeline (frontier_explorer.py:320-321). |
| `w_distance` | `1.0` | float | Weight of the distance-to-preferred scorer. |
| `w_novelty` | `1.0` | float | Weight of the novelty scorer (only when `use_novelty_scoring` is on). |
| `w_clearance` | `1.0` | float | Weight of the clearance scorer; `0.0` disables both the clearance bonus and the clearance floor filter (frontier_explorer.py:323-325, 396-397). |
| `robot_radius` | `0.17` | float | Inscribed radius used for the clearance floor (frontier_explorer.py:291-296). |
| `clearance_margin_m` | `0.05` | float | Clearance floor = `robot_radius + clearance_margin_m`. |

---

## 3. ROS2 parameters — slam manager

Declared in `dome_nav/slam_manager_node.py:22-28`. Runtime-configurable.

| Parameter | Default | What it tunes |
|---|---|---|
| `map_persist_path` | `$DOME_HOME/slam_map` | Pose-graph / legacy map save destination. Launch files set it per-map to `~/.dome/slam_maps/<map_name>`. |
| `export_legacy_map` | `True` | Also export `.pgm`/`.yaml` via `slam_toolbox/save_map`. |

`nav_manager_node` declares no parameters. `hello_world_algorithm` declares its
own `preferred_goal_distance` (step distance, default `1.0`) since the shared
field moved to `FrontierParams` (F34 T03).

---

## 4. Launch-file exposure

Which parameters each launch file actually sets. A `-1` sentinel in `just_explorer`
means "keep the node default".

| Launch file | Explorer params set | Config files used |
|---|---|---|
| `just_explorer.launch.py` | `map_name`, `use_sim_time`, `min_frontier_dist`, `max_frontier_dist`, `min_frontier_size`, `preferred_goal_distance`, `max_explore_radius`, `blacklist_radius`, `use_novelty_scoring`, `w_distance`, `w_novelty`, `w_clearance`, `robot_radius`, `clearance_margin_m` | — (explorer only) |
| `sim_explore_node.launch.py` (duplicated in `sim_nav_full.launch.py:72-81`) | `max_frontier_dist=15.0`, `min_frontier_dist=0.9`, `preferred_goal_distance=2.0`, `blacklist_radius=0.5`, `min_frontier_size=5`, `w_*=1.0`, `robot_radius=0.17`, `clearance_margin_m=0.05`, `max_explore_radius=0.0`, `map_name` (required), `use_sim_time=True` | sim yamls |
| `robot_explore.launch.py` | `max_frontier_dist=0.0`, `min_frontier_dist=0.5`, `preferred_goal_distance=2.0`, `blacklist_radius=0.5`, `frontier_buffer_cells=0`, `min_frontier_size=10`, `w_*=1.0`, `robot_radius=0.17`, `clearance_margin_m=0.05`; sets `slam_manager_node.map_persist_path` | `mapper_params_online_async.yaml`, `nav2_params_explore_real.yaml` |
| `nav_experiment.launch.py` | Mirrors robot_explore minus the `w_*`/`robot_radius`/clearance overrides (node defaults apply); takes `slam_config`/`nav2_config` paths as args | user-supplied |
| `robot_map.launch.py` | — | `mapper_params_online_async.yaml`, `nav2_params_real.yaml` |
| `robot_nav.launch.py` | — | `nav2_params_localization_real.yaml` + `nav2_params_real.yaml`, static map `~/.dome/slam_maps/basement1.yaml` |
| `sim_slam.launch.py` / `sim_nav2.launch.py` | — | `mapper_params_online_async_sim.yaml` / `nav2_params_explore_sim.yaml` |
| `nav2_experiment_navigation.launch.py` | — | All tuning via `params_file` arg (trimmed nav2_bringup) |
| `sim_robot.launch.py` | `world_name`, `urdf_name="minimal_sim.urdf"`, spawn z=0.05 | — |

---

## 5. YAML configs

Exploration-relevant values in `config/*.yaml` that differ from upstream nav2 /
slam_toolbox defaults. All runtime-configurable via launch-file selection.

### slam_toolbox

| File | Key tuned values |
|---|---|
| `mapper_params_online_async.yaml` (real) | `resolution 0.05`, `max_laser_range 10.0`, `map_update_interval 10.0`, `transform_publish_period 0.05`, `transform_timeout 0.5`, `tf_buffer_duration 50.0`, `message_queue_size 40`, `minimum_travel_distance 0.5`, `minimum_travel_heading 0.5`, `map_start_at_dock true` |
| `mapper_params_online_async_sim.yaml` (sim) | Same shape but `map_update_interval 1.0`, `transform_publish_period 0.02`, `message_queue_size 20`, `minimum_travel_distance 0.1`, `minimum_travel_heading 0.1` — faster/finer map growth for early exploration |

### nav2

| File | Key tuned values |
|---|---|
| `nav2_params_explore_sim.yaml` | goal_checker `xy_goal_tolerance 0.5`, `yaw_goal_tolerance 3.15`; progress_checker `required_movement_radius 0.3`, `movement_time_allowance 30.0`; MPPI `batch_size 1000`, `vx_max/min ±0.45`, `az_max 3.2`; costmaps `robot_radius 0.15`, local 5×5 m `cost_scaling_factor 30.0`, global `lethal_cost_threshold 65`, `unknown_cost_value -1`, obstacle_layer dropped, `inflation_radius 0.7`; velocity_smoother max `[0.6, 0, 1.9]`; collision_monitor `time_before_collision 1.2` (FootprintApproach disabled) |
| `nav2_params_explore_real.yaml` (real explore) | `yaw_goal_tolerance 1.0`; MPPI `vx_max/min ±0.6`, `wz_max 1.4`, `batch_size 1000`; planner = **SmacPlanner2D**; costmaps `robot_radius 0.15`, local `inflation_radius 0.25`, global `inflation_radius 0.2`, `lethal_cost_threshold 65`, `unknown_cost_value -1`, obstacle_layer dropped; rotation_shim `max_rotational_vel 1.4` / `min_rotational_vel 0.3`; velocity_smoother max `[0.6, 0, 1.4]` |
| `nav2_params_explore_real_mini.yaml` | Upstream-verbatim annotated variant: MPPI `time_steps 36`, `vx_max/min ±0.25`; costmap `robot_radius 0.17`, local `inflation_radius 0.4`, global `cost_scaling_factor 2.5`; velocity_smoother `±0.25` linear; collision_monitor `time_before_collision 1.0` |
| `nav2_params_real.yaml` (Mode A/B nav) | critic `threshold_to_consider 0.5`; `lethal_cost_threshold 65`, `unknown_cost_value -1` |
| `nav2_params_localization_real.yaml` (Mode B) | costmap `robot_radius 0.15`; local `cost_scaling_factor 30.0` / `inflation_radius 0.17`, global 15.0 / 0.3; `lethal_cost_threshold 65` |

---

## 6. Hard-coded constants

These require a code edit to change.

### `explorer_manager_node.py:65-89` — session / watchdog behavior

| Constant | Value | What it controls |
|---|---|---|
| `EXPLORE_HZ` | `1.0` | Exploration tick rate |
| `NO_TARGET_PATIENCE` | `14` | Consecutive no-goal ticks before the session is declared done |
| `GOAL_TIMEOUT_S` | `25.0` | Cancel goal to break Nav2 BT recovery loops |
| `STUCK_T_S` | `20.0` | No-progress abandon timeout (deliberately > Nav2 progress_checker's 10 s) |
| `STUCK_MOVE_EPS` | `0.05` | Meters moved that count as progress |
| `STUCK_PROGRESS_EPS` | `0.10` | Distance-to-goal drop that counts as progress |
| `MAX_GOAL_ATTEMPTS` | `8` | Per-tick goal candidate retries past out-of-costmap / lethal candidates |
| `WEDGED_STUCK_LIMIT` | `3` | Same-pose stuck failures before stopping as wedged |
| `DEFAULT_ALGORITHM` | `"frontier"` | Fallback algorithm |

### Elsewhere

- `frontier_explorer.py:19` — `OCCUPIED_THRESHOLD = 65`: /map cell value treated as wall for the clearance field.
- `explore_diagnostics.py:19-23` — `LETHAL_COST = 100`, `INSCRIBED_COST = 99`, `LETHAL_THRESHOLD = 99`: scaled-costmap cost at which a goal is rejected; `costmap_radius_costs(radius_cells=4)`.
- `nav_manager.py:28-29` — `MAX_COV = 1.0` ("lost" covariance ceiling), `CONVERGED_THRESHOLD = 0.9` (AMCL score → converged/localizing).
- `utils.py:39-42` — `WORLD_SPAWN_XY` sim spawn poses per world; `DOME_HOME` default `~/.dome` (utils.py:13).
- `explore_telemetry.py:13-14` — telemetry dir hard-coded `~/.dome/telemetry` (ignores the `DOME_HOME` override used elsewhere); filename capped at 32 chars.

---

## 7. Architecture notes

- **Two-layer params**: `ExploreParams` (shared, node-owned) + `FrontierParams`
  (algorithm-owned) are merged each tick into `FrontierTuning` by `merge_tuning`
  (frontier_params.py), which the pure functions in `frontier_explorer.py` consume.
  Constraint: `blacklist_radius > goal_inset_m` is enforced at merge time. The
  shared overlay is exactly two fields — `max_explore_radius`, `blacklist_radius`
  (F34 T03 ownership rule: shared iff the node itself reads it).
- **Algorithm registry**: `ALGORITHM_REGISTRY` (explorer_manager_node.py:47-51) maps
  `explore_algorithm` → class. Each algorithm self-declares its ROS params through the
  `declare_params(node)` protocol hook (explore_context.py:102-107). To add a tunable
  per-algorithm knob, add a field to `FrontierParams` and a matching entry in
  `declare_frontier_params()` — the launch/yaml plumbing then works automatically.
- **Known quirks**:
  - `blacklist_radius` is now a declared ROS param (F34 T02), exposed in
    `robot_explore`, `sim_explore_node`, `just_explorer`, `nav_experiment`;
    `sim_nav_full` leaves it at the default.
  - `goal_inset_m` is ROS-declared but no launch file sets it.

---

## 8. Launch-file inventory & redundancy

All files in `launch/`. **Composed** = brings up a full stack (often via `bl.include`);
**leaf** = one piece meant to be combined with others.

### Real robot

| File | Kind | Purpose |
|---|---|---|
| `robot_explore.launch.py` | composed | **Mode A** — slam_toolbox + Nav2 + explorer for autonomous map building. Primary real-robot exploration entry point. |
| `robot_map.launch.py` | composed | slam_toolbox + Nav2 + dome_nav nodes, **no explorer** — manual (teleop) map building. |
| `robot_nav.launch.py` | composed | **Mode B** — static saved map + AMCL + Nav2 for normal operation. Needs a map built by `robot_map`. |
| `nav_experiment.launch.py` | composed | Experiment harness: slam + Nav2 + optional explorer (when `--map_name` given), both config yamls passed as args. Assumes driver stack runs separately. |
| `nav2_experiment_navigation.launch.py` | leaf | Trimmed fork of nav2_bringup `navigation_launch.py` — drops 3 unused lifecycle servers to save Pi CPU. Included by the experiment path, not run directly. |

### Simulation

| File | Kind | Purpose |
|---|---|---|
| `sim_nav_full.launch.py` | composed | Single-command full sim stack; includes the four sim leaves in dependency order. **Everyday sim entry point.** |
| `sim_robot.launch.py` | leaf | Gazebo + spawn + bridge + RSP + laser TF. Visible TF-correct robot, no slam/Nav2. |
| `sim_slam.launch.py` | leaf | slam_toolbox online_async; split from sim_nav so `/map` can be confirmed before Nav2. |
| `sim_nav2.launch.py` | leaf | Nav2 stack; split so it starts only after `/map` exists (else lifecycle abort). |
| `sim_explore_node.launch.py` | leaf | explorer_manager_node only; needs sim_robot + sim_slam + sim_nav2 already up. Manual-debug counterpart of sim_nav_full's explorer. |
| `sim_rviz.launch.py` | leaf | RViz2 with `use_sim_time`. Optional viz. |
| `just_explorer.launch.py` | leaf | explorer_manager_node only, bring-your-own `/map` + Nav2. Sim/real-agnostic; all tuning via explicit launch args. |

### Redundancy assessment

- **`sim_explore_node.launch.py` ⇄ `sim_nav_full.launch.py:72-81` — real duplication.**
  The explorer param block is copy-pasted in both (noted in §4). Only `sim_explore_node`'s
  header even admits it ("same sim-only exploration defaults as sim_nav_full"). Change one,
  the other drifts. **Candidate for dedupe**: factor the param dict into a shared helper both
  include, or have `sim_nav_full` `bl.include(sim_explore_node)` like it does the other leaves.
- **`just_explorer` vs `sim_explore_node`** — near-overlap (both = explorer-only). Not
  redundant: `just_explorer` takes tuning as explicit args and is stack-agnostic;
  `sim_explore_node` hard-codes sim defaults. Keep both, but `sim_explore_node` could become
  a thin `just_explorer` wrapper.
- **`robot_explore` vs `nav_experiment`** — overlap (both = slam + Nav2 + explorer). Not
  redundant: `nav_experiment` swaps configs via args for A/B testing; `robot_explore` is the
  fixed production stack. Purposeful split.
- **`robot_map` vs `robot_explore`** — same node set minus the explorer. Distinct purpose
  (manual vs autonomous mapping). Keep.

**Verdict**: one genuine redundancy — the duplicated sim explorer params
(`sim_explore_node` ⇄ `sim_nav_full`). Everything else is intentional
leaf/composed layering from the F13 T04 split.
