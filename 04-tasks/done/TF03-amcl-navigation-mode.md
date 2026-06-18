# TF03 — AMCL Navigation Mode (Feature F03)

## T01 — rename robot.launch.py to robot_map.launch.py
**Status**: done
**Description**: Mode A (map building). Rename existing `launch/robot.launch.py` to
`launch/robot_map.launch.py`. No logic changes — slam_toolbox + Nav2 + manager nodes unchanged.
Update `setup.py` data_files entry if it lists launch files by name.
**Test**: `colcon build --packages-select dome_nav` passes. File exists at new path.

## T02 — create config/nav2_amcl_patch.yaml
**Status**: done
**Description**: AMCL params tuned for dome robot. Patch applied on top of nav2_bringup
`nav2_params.yaml`. Must include: amcl laser/odom model, particle counts, update thresholds.
map_server section: topic `/map`, frame `map`. Do NOT include slam_toolbox params.
**Test**: yaml is valid (python -c "import yaml; yaml.safe_load(open(...))").

## T03 — create launch/robot_nav.launch.py
**Status**: done
**Description**: Mode B (navigation). Loads saved map via `nav2_bringup localization_launch.py`
(map_server + AMCL), then `nav2_bringup navigation_launch.py` (planner + controller + costmap).
Includes `nav_manager_node`. Does NOT include `slam_manager_node` or slam_toolbox.
Map path: `~/.dome/slam_maps/basement1.yaml`. Params: `nav2_amcl_patch.yaml` merged over
`nav2_params.yaml` for both localization and navigation launches.
**Test**: `colcon build` passes. File importable by launch system.

## T04 — update setup.py to export new launch files
**Status**: done (setup.py uses glob — no change needed; both files auto-included)
**Description**: Ensure `setup.py` `data_files` includes both `robot_map.launch.py` and
`robot_nav.launch.py`. Remove old `robot.launch.py` entry if present.
**Test**: `colcon build` passes. Both launch files present in `install/dome_nav/share/`.

## T05 — manual smoke test
**Status**: done
**Description**: With saved map at `~/.dome/slam_maps/basement1.yaml`, launch Mode B:
`bl dome_nav robot_nav.launch.py`. Verify in RViz/Foxglove: map appears, AMCL particle
cloud visible, `map→odom` TF published by amcl (not slam_toolbox).
**Test**: manual observation. Cannot be automated without full robot stack.
