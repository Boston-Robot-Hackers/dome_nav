# Session Handoff — 2026-07-09

## What we did this session

1. **`prefer_farthest` → `True`** in `launch/robot_explore.launch.py` (real robot)

2. **Nav failure diagnostics** added to `pluggable_explore_manager_node.py`:
   - `dump_failure_diagnostics()` — prints costmap heatmaps (4-cell radius around goal/robot), blacklist, frontier clusters on NAV2 abort
   - `dump_frontier_exhaustion()` — cluster filter breakdown when patience timer expires
   - `paused_on_failure` flag — exploration halts on abort, resumes via `exploration_resume` intent
   - Subscribed to `/global_costmap/costmap` and `/local_costmap/costmap`
   - `NAV2_ERROR_CODES` lookup table (100/200 range)

3. **Telemetry** switched from `explore-{map_name}-{date}.jsonl` (append) to sequential `exp-{NNNN}.json` (one file per run, write mode)

4. **MPPI crawl fix** — `GoalCritic.threshold_to_consider` and `PathFollowCritic.threshold_to_consider` changed `1.4 → 0.5` in all 3 active configs: `nav2_params_explore_real.yaml`, `nav2_params_explore_sim.yaml`, `nav2_params_real.yaml`

5. **Costmap fixes** across all configs:
   - `nav2_params_real.yaml`: moved `lethal_cost_threshold: 65` / `unknown_cost_value: -1` / `trinary_costmap: true` from under `static_layer` (dead/ignored) to top-level global_costmap (now effective)
   - `nav2_params_localization_real.yaml` (Mode B, previously unaudited): `robot_radius 0.22→0.15`, local inflation `0.7/3.0→0.17/30.0`, global inflation `0.7/3.0→0.3/15.0`, added `lethal_cost_threshold: 65` / `unknown_cost_value: -1`
   - All configs: global `inflation_radius 0.5→0.3` to reduce `PLAN/GOAL_OCCUPIED` near walls

## Current git state

Branch: `main`. All committed and pushed. Latest commit: `1d46119`.

## What failed

User tried to run sim and it "failed immediately." Screenshot was on Mac desktop, didn't transfer. Error text not yet known. Need to get the actual error output.

## Remaining tuning suggestions (not yet done)

From the Nav2 tuning analysis:
- Fix dead keys in `nav2_params_real.yaml` for `desired_linear_vel` / `max_angular_accel` (MPPI ignores them — cosmetic cleanup only)
- `CostCritic.consider_footprint: false → true` (safer in tight spaces, more CPU)
- `MPPI.batch_size: 2000 → 4000` (better trajectories, more CPU)
- Re-enable `FootprintApproach` in `nav2_params_explore_real.yaml` (currently `enabled: false` for diagnostic isolation)
- Add `obstacle_layer` to global_costmap in explore configs (currently only static+inflation)
- `SmacPlanner2D.use_astar: false → true` (faster long-range planning)
- `MPPI.visualize: true → false` for production

## Sim launch commands

```bash
# Terminal 1
gz sim -r ~/ros2_ws/install/dome_nav/share/dome_nav/worlds/multi_room.world

# Terminal 2 — MUST build first
colcon build --packages-select dome_nav
bl dome_nav sim_nav_full.launch.py --map_name test1 --world_name multi_room

# Terminal 3 (optional)
bl dome_nav sim_rviz.launch.py

# Terminal 4 — start exploration
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_start\", \"source\": \"cli\", \"slots\": {}}"'

# Watch
ros2 topic echo /explore/status
tail -f ~/.dome/telemetry/exp-*.json

# If paused on abort
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_resume\"}"'
```

## Key gotchas

- **Must `colcon build` after every source/YAML edit** before `bl` sees it
- Stale nodes across runs → TF/clock collisions → `ps` audit + `kill -9`
- `nav2_params_localization_real.yaml` changes (Mode B) need hardware verification — never live-run
- `FootprintApproach` still disabled in explore config — restore when done with crawl isolation
