# TF25 — Minimal Real-Robot Explore Config for F25

## T01 — Create nav2_params_explore_real_mini.yaml
**Status**: done
**Description**: Copy upstream `nav2_bringup/params/nav2_params.yaml` verbatim, add a
provenance header, and apply the three surgical deltas: `robot_radius` 0.22→0.15 (both
costmaps), `FootprintApproach.time_before_collision` 1.2→0.5 (comment UNVERIFIED),
`deadband_velocity` left [0,0,0] with a Bug-1-critical comment. No other changes.
**Test**: n/a (config). Sanity: `python3 -c "import yaml; yaml.safe_load(open(...))"`
parses clean; diff vs upstream shows only the three intended deltas.

## T02 — Live verification
**Status**: not done — hardware, deferred
**Description**: Run the harness with `--nav2_config .../nav2_params_explore_real_mini.yaml`
on the real robot; confirm the robot drives (E0-equivalent baseline holds with radius
0.15) and record whether tbc 0.5 changes collision_monitor braking vs 1.2. Folds into
the E6/E7 experiment runs (still pending).
