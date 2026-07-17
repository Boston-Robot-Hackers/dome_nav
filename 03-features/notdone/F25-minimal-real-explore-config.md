# F25 — Minimal Real-Robot Explore Nav2 Config

**Priority**: Medium
**Done:** no
**Tasks File Created:** yes (TF25)
**Tests Written:** n/a (config file)
**Test Passing:** n/a
**Description**: Add `config/nav2_params_explore_real_mini.yaml` — a copy of upstream
`nav2_bringup/params/nav2_params.yaml` with ONLY surgical, justified deltas, as an
alternative to the large hand-forked `nav2_params_explore_real.yaml` (400-line rewrite
with dozens of unvetted overrides). Goal: start from the known-good E0 baseline
(robot moved on pure upstream) and layer the smallest set of changes.

## Deltas from upstream (and provenance)

| Change | From → To | Provenance |
|---|---|---|
| `robot_radius` (local + global costmap) | 0.22 → 0.15 | robot-physical (true DOME radius); also the lever on Bug-2's ≥253 lethal ring |
| `FootprintApproach.time_before_collision` | 1.2 → 0.5 | E6/E7 **designed, NOT yet run** — marked UNVERIFIED in-file |
| `deadband_velocity` | [0,0,0] (unchanged) | Bug 1 SOLVED requires [0,0,0]; upstream already correct — commented, not changed |

Deliberately NOT carried from `nav2_params_explore_real.yaml`: SmacPlanner2D (kept
upstream NavFn), relaxed goal tolerances, batch_size 1000, inflation/cost_scaling
overrides, map_topic additions — none experiment-proven, all left at upstream.

## Constraints

- Frames/topics already match upstream (base_footprint/base_link/odom, scan,
  cmd_vel_smoothed→cmd_vel) — nothing to change there.
- collision_monitor stays ENABLED (E5's disable is unproven and frees only Bug-2 gate 3).
- Not wired into any launch by default; opt-in via `--nav2_config`.

## How to Demo

```
bl dome_nav nav_experiment.launch.py \
    --slam_config /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml \
    --nav2_config /home/pitosalas/ros2_ws/src/dome_nav/config/nav2_params_explore_real_mini.yaml \
    --map_name minitest
```
Compare drive behavior vs the full `nav2_params_explore_real.yaml`.
