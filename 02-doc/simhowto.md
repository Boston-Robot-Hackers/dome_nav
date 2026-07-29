# Sim Exploration Test How-To

Quick reference for running the F31 frontier-exploration comparison on the VM.
Assumes ROS 2 Kilted on Ubuntu 24.04 and the dome_nav workspace already built.

## One-time prep

```bash
cd /home/pitosalas/ros2_ws/src/dome_nav
git pull
colcon build --packages-select dome_nav
source /home/pitosalas/ros2_ws/install/setup.bash
mkdir -p ~/.dome/telemetry
```

## Environment (every terminal)

```bash
source /home/pitosalas/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
```

Gazebo needs software rendering on the VM; without it the GUI flickers and
restarts.

## Launch order

Open one terminal per step.

### 1. Gazebo + robot + bridge + TF

```bash
source /home/pitosalas/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
bl dome_nav sim_robot.launch.py --world_name multi_room
```

Wait until Gazebo is stable and these topics are publishing:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /clock
```

### 2. SLAM

```bash
source /home/pitosalas/ros2_ws/install/setup.bash
bl dome_nav sim_slam.launch.py
```

Wait for `/map` and the `map→odom` transform:

```bash
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

### 3. Nav2

```bash
source /home/pitosalas/ros2_ws/install/setup.bash
bl dome_nav sim_nav2.launch.py
```

Wait until all lifecycle nodes are `active`:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

Also confirm the action server is up:

```bash
ros2 action list
ros2 action info /navigate_to_pose
```

### 4. RViz (optional but useful)

```bash
source /home/pitosalas/ros2_ws/install/setup.bash
rviz2
```

In RViz:
- Set **Fixed Frame** to `map`.
- Add a **MarkerArray** display, topic `/explore/markers`.
- Add a **LaserScan** display, topic `/scan`.
- Add a **Map** display, topic `/map`.

### 5. Explorer

#### Session A — baseline (`w_clearance = 0.0`)

```bash
source /home/pitosalas/ros2_ws/install/setup.bash
bl dome_nav just_explorer.launch.py \
  --map_name f31_base \
  --use_sim_time true \
  --min_frontier_dist 0.9 \
  --max_frontier_dist 15.0 \
  --min_frontier_size 5 \
  --preferred_goal_distance 2.0 \
  --w_clearance 0.0
```

#### Session B — tuned (`w_clearance = 1.0`)

```bash
source /home/pitosalas/ros2_ws/install/setup.bash
bl dome_nav just_explorer.launch.py \
  --map_name f31_clear \
  --use_sim_time true \
  --min_frontier_dist 0.9 \
  --max_frontier_dist 15.0 \
  --min_frontier_size 5 \
  --preferred_goal_distance 2.0
```

`w_clearance` defaults to `1.0`, so omitting it enables clearance scoring.

### 6. Start exploration

```bash
ros2 topic pub --once /intent std_msgs/msg/String 'data: "{\"name\": \"exploration_start\", \"source\": \"cli\", \"slots\": {}}"'
```

## Watch while it runs

```bash
# Status and goals
ros2 topic echo /explore/status

# Collision-monitor gate events (count FootprintApproach entries)
ros2 topic echo /collision_monitor_state

# Telemetry (use the active filename from ~/.dome/telemetry/)
tail -f ~/.dome/telemetry/ef31_base*.json
tail -f ~/.dome/telemetry/ef31_clear*.json
```

## Cleanup between sessions

```bash
pkill -f "ros2|gz|rviz2"
```

Then restart from step 1.

## Common problems

- **`OdomSmoother has not received any data yet`**: `/odom` is not publishing or
  `bt_navigator` started before Gazebo. Check `ros2 topic hz /odom` and wait
  longer between steps 1 and 3.

- **`NavigateToPose server not ready`**: Nav2 lifecycle is still activating or
  `bt_navigator` crashed. Check `ros2 lifecycle get /bt_navigator` and the
  `sim_nav2` terminal for errors.

- **No explore markers in RViz**: Fixed Frame must be `map`; add a MarkerArray
  display on `/explore/markers`. Verify with `ros2 topic hz /explore/markers`.

- **Gazebo flickers / blank**: `export LIBGL_ALWAYS_SOFTWARE=1` is required on
  the VM.
