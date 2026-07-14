# dome_nav — Spec

## Purpose

dome_nav is the navigation infrastructure for the DOME robot. It owns SLAM, map
persistence, localization, and Nav2 configuration. It exposes a stable
intent-driven navigation interface to the rest of the dome stack.

It operates in two explicit modes:

- **Map-build mode** — human teleops robot, slam_toolbox builds and saves occupancy map
- **Navigation mode** — AMCL localizes against saved map, Nav2 handles path planning

Other packages (dome_control, dome_vision) are not required for dome_nav to function.

---

## Real robot velocity limits (hardware spec)

These are the true, measured velocity capabilities of the physical DOME robot.
They are the source of truth for the velocity limits and deadbands in the
`nav2_params_*_real.yaml` configs (MPPI `vx_max`/`vx_min`/`wz_max`,
`velocity_smoother` `max_velocity`/`min_velocity`/`deadband_velocity`, and
`behavior_server` rotational limits). Keep those configs in sync with this table.

| Axis | Min (motor floor) | Max |
|------|-------------------|-----|
| Linear (forward/backward) | 0.1 m/s | 0.6 m/s |
| Angular (turning)         | 0.3 rad/s | 1.4 rad/s |

- **Min** = the slowest speed the robot can actually execute; commands below this
  are below the motor/static-friction threshold and produce no motion (they must
  be zeroed via the velocity_smoother `deadband_velocity`, not sent to the base).
- **Max** = the hardware ceiling; MPPI and the velocity_smoother caps must agree
  and must not exceed these.

---

## Mode A — Map Build (`robot_map.launch.py`)

Run once (or when house layout changes significantly).

**Launches:**
- `slam_toolbox` online_async
- `slam_manager_node` — monitors `/map`, saves pose graph on shutdown

**Inputs:**
- `/scan` (LaserScan) — from robot lidar
- `/odom` (Odometry) — from robot

**Outputs:**
- `/map` (OccupancyGrid)
- `/dome_nav/slam_status` (String) — `"waiting"` | `"mapping"`
- Saves `~/.dome/slam_map.posegraph` + `~/.dome/slam_map.pgm` + `~/.dome/slam_map.yaml`

**No Nav2. No localization. No dome_vision. No dome_control required.**

---

## Mode B — Navigation (`robot_nav.launch.py`)

Normal operating mode. Requires a saved map from Mode A.

**Launches:**
- `map_server` — serves saved occupancy grid
- `amcl` — particle-filter localization against static map
- Nav2 (planner + controller + costmap) — no map_server or amcl in nav2 bringup
- `nav_manager_node` — bridges `/intent` → `NavigateToPose`, publishes status

**Inputs:**
- `/scan` (LaserScan) — AMCL needs lidar to localize
- `/odom` (Odometry) — robot odometry
- `/navigate_to_pose` (Nav2 action) — from any caller
- `/intent` (String JSON) — from dome_control (optional; nav_manager handles it)
- `/targets/confirmed` (String JSON) — from dome_vision (optional; goto needs it)

**Outputs:**
- `/dome_nav/nav_status` (String) — `"idle"` | `"navigating:<label>"` | `"cancelled"` | `"done"` | `"failed"`
- `/cmd_vel` (Twist) — Nav2 → robot

**AMCL global localization:** converges from lidar alone. No initial pose needed.
No fiducials required. No fixed start position.

---

## Localization — swappable for outdoor use

dome_nav's localization source is a launch-time choice, not an API:

| Environment | Localization | Config |
|-------------|-------------|--------|
| Indoor | AMCL (particle filter, lidar) | `robot_nav.launch.py` |
| Outdoor (future) | `robot_localization` EKF + GPS/RTK + IMU | `robot_nav_outdoor.launch.py` |

Nav2, costmap, NavigateToPose action, and `/dome_nav/nav_status` are identical in
both cases. dome_control and dome_vision do not change.

---

## Topics

### Published
- `/dome_nav/slam_status` (String) — Mode A only
- `/dome_nav/nav_status` (String) — Mode B only

### Subscribed
- `/map` (OccupancyGrid) — from slam_toolbox (Mode A) or map_server (Mode B)
- `/intent` (String JSON) — from dome_control, optional
- `/targets/confirmed` (String JSON) — from dome_vision, optional

---

## Map Persistence

- Pose graph: `~/.dome/slam_map.posegraph` + `~/.dome/slam_map.data`
- Occupancy grid: `~/.dome/slam_map.yaml` + `~/.dome/slam_map.pgm`
- AMCL loads occupancy grid (`map.yaml` + `map.pgm`) at startup in Mode B
- `map_start_at_dock` **removed** — AMCL converges from any start position

---

## Testability

**Without dome_vision or dome_control:**
- Mode A: run with lidar + odom only. Verify map builds and saves.
- Mode B: run with lidar + odom + rosbag. Verify AMCL converges, `/dome_nav/nav_status` publishes.
- Send `NavigateToPose` directly via `ros2 action` CLI — no dome_control needed.

**Without ROS (unit tests):**
- Manager node logic extracted into pure Python classes (`slam_manager.py`, `nav_manager.py`)
- ROS nodes are thin wrappers around these classes
- Tests mock ROS interfaces; no rclpy dependency in core logic

---

## Interfaces with other dome packages

| Package | Interface | Required? |
|---------|-----------|-----------|
| dome_control | Sends `/intent`; receives `/dome_nav/nav_status` | No |
| dome_vision | dome_nav provides TF chain (map→odom→base_link); dome_vision uses it to pin objects to map frame | No |
| linorobot2 | Provides URDF, TF tree, odometry, lidar scan | Yes |
