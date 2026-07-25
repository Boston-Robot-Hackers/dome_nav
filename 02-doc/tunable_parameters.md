# Tunable Parameters Guide

This document inventories every parameter you can use to tune the exploration and
navigation behavior of `dome_nav`, grouped by where it lives:

1. [ROS2 parameters — explorer manager (shared)](#1-ros2-parameters-explorer-manager-shared)
2. [ROS2 parameters — frontier algorithm](#2-ros2-parameters-frontier-algorithm)
3. [ROS2 parameters — slam manager](#3-ros2-parameters-slam-manager)
4. [Launch-file exposure](#4-launch-file-exposure)
5. [YAML configs (slam_toolbox / nav2)](#5-yaml-configs)
6. [Hard-coded constants (code-edit only)](#6-hard-coded-constants)
7. [Architecture notes](#7-architecture-notes)

**Runtime-configurable** means the value can be set in a launch file or YAML and changed
without editing code. **Code-edit only** means the default is baked into the source.

---

## 1. ROS2 parameters — explorer manager (shared)

Declared in `dome_nav/explorer_manager_node.py:116-119`, read into `ExploreParams`
(`dome_nav/explore_context.py:49-59`). Owned by the node, shared by all algorithms.

| Parameter | Default | Type | What it tunes |
|---|---|---|---|
| `explore_algorithm` | `"frontier"` | str | Algorithm selection from `ALGORITHM_REGISTRY` (`frontier` / `hello`, explorer_manager_node.py:47-51). Unknown name → warning + fallback. |
| `max_explore_radius` | `0.0` (unlimited) | float | Radius from session start pose beyond which frontier clusters are filtered out (frontier_explorer.py:303-308). |
| `preferred_goal_distance` | `1.0` | float | Target travel distance per goal; scoring minimizes \|actual reach − preferred\| (frontier_explorer.py:269-273). |
| `map_name` | `"unknown"` | str | Telemetry filename tag only (`~/.dome/telemetry/e<map><date>.json`); no effect on mapping. |

Shared tuning that is **not** a ROS parameter (code-edit only, `explore_context.py:49-59`):

- `blacklist_radius = 0.5` — exclusion radius around failed goals (kept `> goal_inset_m`
  by `merge_tuning`, frontier_params.py:51-63). This is the one shared knob with no ROS
  or launch exposure anywhere; robot_explore.launch.py:53-61 notes this is deliberate.

---

## 2. ROS2 parameters — frontier algorithm

Defaults live in the `FrontierParams` dataclass (`dome_nav/frontier_params.py:11-27`);
each is declared via `declare_frontier_params()` (frontier_params.py:90-123) when the
algorithm registers itself. All runtime-configurable.

### Goal / frontier filtering

| Parameter | Default | Type | What it tunes |
|---|---|---|---|
| `min_frontier_size` | `15` | int | Drop frontier clusters smaller than this many cells (frontier_explorer.py:299-300). |
| `min_frontier_dist` | `1.3` | float | Goal must be at least this far from the robot (0 disables). |
| `max_frontier_dist` | `0.0` (unlimited) | float | Upper bound on robot-to-goal distance (0 disables). |
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

### Deprecated

| Parameter | Default | Status |
|---|---|---|
| `prefer_farthest` | `False` | Deprecated; maps to farthest-first selection via `preferred_goal_distance` override (frontier_params.py:64-69). |
| `novelty_top_n` | `5` | Deprecated no-op (two-stage shortlist retired in F31); still logged in session params. |

---

## 3. ROS2 parameters — slam manager

Declared in `dome_nav/slam_manager_node.py:22-28`. Runtime-configurable.

| Parameter | Default | What it tunes |
|---|---|---|
| `map_persist_path` | `$DOME_HOME/slam_map` | Pose-graph / legacy map save destination. Launch files set it per-map to `~/.dome/slam_maps/<map_name>`. |
| `export_legacy_map` | `True` | Also export `.pgm`/`.yaml` via `slam_toolbox/save_map`. |

`nav_manager_node` and `hello_world_algorithm` declare no parameters.

---

## 4. Launch-file exposure

Which parameters each launch file actually sets. A `-1` sentinel in `just_explorer`
means "keep the node default".

| Launch file | Explorer params set | Config files used |
|---|---|---|
| `just_explorer.launch.py` | `map_name`, `use_sim_time`, `min_frontier_dist`, `max_frontier_dist`, `min_frontier_size`, `preferred_goal_distance`, `max_explore_radius`, `use_novelty_scoring`, `novelty_top_n`, `w_distance`, `w_novelty`, `w_clearance`, `robot_radius`, `clearance_margin_m` | — (explorer only) |
| `sim_explore_node.launch.py` (duplicated in `sim_nav_full.launch.py:72-81`) | `max_frontier_dist=15.0`, `min_frontier_dist=0.9`, `preferred_goal_distance=2.0`, `min_frontier_size=5`, `w_*=1.0`, `robot_radius=0.17`, `clearance_margin_m=0.05`, `max_explore_radius=0.0`, `map_name` (required), `use_sim_time=True` | sim yamls |
| `robot_explore.launch.py` | `max_frontier_dist=0.0`, `min_frontier_dist=0.5`, `preferred_goal_distance=2.0`, `frontier_buffer_cells=0`, `min_frontier_size=10`, `w_*=1.0`, `robot_radius=0.17`, `clearance_margin_m=0.05`; sets `slam_manager_node.map_persist_path` | `mapper_params_online_async.yaml`, `nav2_params_explore_real.yaml` |
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
  (frontier_params.py:30-87), which the pure functions in `frontier_explorer.py` consume.
  Constraint: `blacklist_radius > goal_inset_m` is enforced at merge time.
- **Algorithm registry**: `ALGORITHM_REGISTRY` (explorer_manager_node.py:47-51) maps
  `explore_algorithm` → class. Each algorithm self-declares its ROS params through the
  `declare_params(node)` protocol hook (explore_context.py:102-107). To add a tunable
  per-algorithm knob, add a field to `FrontierParams` and a matching entry in
  `declare_frontier_params()` — the launch/yaml plumbing then works automatically.
- **Known quirks**:
  - `blacklist_radius` is the only shared tuning with no ROS/launch exposure.
  - `goal_inset_m` is ROS-declared but no launch file sets it.
