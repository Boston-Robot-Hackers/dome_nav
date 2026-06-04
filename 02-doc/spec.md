# dome_nav — Spec

## Purpose

dome_nav wraps slam_toolbox and Nav2 for the DOME robot. It owns all SLAM and
navigation configuration, manages map persistence across sessions, and exposes
a simple intent-driven navigation interface to the rest of the dome stack.

## Responsibilities

1. **SLAM** — run slam_toolbox online_async. Build map from scratch on first run.
   Load saved pose graph on subsequent runs. Publish `map → odom` TF correction.

2. **Navigation** — run Nav2 with the slam_toolbox map. Accept `NavigateToPose`
   goals in `map` frame. Handle obstacle avoidance via lidar costmap.

3. **Map persistence** — serialize the slam_toolbox pose graph to
   `~/.dome/slam_map` on clean shutdown. Save Nav2 occupancy grid alongside it.
   Load on next startup automatically.

4. **Intent bridge** — subscribe to `/intent` (from dome_control). On
   `go_to_object` intent, look up the target in `/targets/confirmed`, compute a
   goal pose in `map` frame, send to Nav2.

## Topics

### Published
- `/dome_nav/slam_status` (`std_msgs/String`) — "waiting" | "mapping"
- `/dome_nav/nav_status` (`std_msgs/String`) — "navigating:<label>" | "cancelled" | "no_target:<label>"

### Subscribed
- `/map` (`nav_msgs/OccupancyGrid`) — from slam_toolbox
- `/intent` (`std_msgs/String`) — JSON intent from dome_control
- `/targets/confirmed` (`std_msgs/String`) — JSON confirmed targets from dome_vision

## Map Persistence

- Pose graph: `~/.dome/slam_map.posegraph` + `~/.dome/slam_map.data`
- Occupancy grid: `~/.dome/slam_map.yaml` + `~/.dome/slam_map.pgm`
- `map_start_at_dock: true` — robot must boot at the same physical origin each session.

## Interfaces with other dome packages

| Package | Interface |
|---|---|
| dome_control | Sends `/intent`; receives `/dome_nav/nav_status` |
| dome_vision | Publishes `/targets/confirmed`; WorldTracker TF frame should be `map` |
| linorobot2 | Provides URDF, TF tree, odometry, lidar scan |
