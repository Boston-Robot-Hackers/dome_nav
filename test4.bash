#!/usr/bin/env bash
# test4.bash — Full sim_explore.launch.py stack with the Gazebo GUI (headless mode
# removed from sim_explore.launch.py — GUI is needed to visually inspect costmap
# inflation and robot behavior near obstacles, e.g. the interior doorway) plus
# foxglove_bridge for remote/alternate monitoring alongside the GUI.
#
# Usage: ./test4.bash [map_name]   (defaults to "sim_test")
# Then in Foxglove Studio: Open Connection -> ws://localhost:8765

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_NAME="${1:-sim_test}"
export LIBGL_ALWAYS_SOFTWARE=1

# sim_explore.launch.py is decorated with ui=True (better_launch terminal UI). The
# TUI needs a real attached terminal; running it backgrounded/headless here breaks
# it and crashes the whole launch ("cannot schedule new futures after interpreter
# shutdown"), orphaning the child nodes it already started. BL_UI_OVERRIDE=disable
# is better_launch's documented escape hatch for headless runs.
export BL_UI_OVERRIDE=disable

source /opt/ros/jazzy/setup.bash
source /home/pitosalas/ros2_ws/install/setup.bash

echo "Killing any stale processes from a previous run..."
# NOTE: robot_state_publisher's cmdline embeds the whole URDF (several KB) as a
# single argument; pkill -f can truncate long cmdlines, so patterns anchored on
# text *after* the URDF blob (e.g. "robot_state_publisher.*dome2") can silently
# fail to match. Match only on the stable binary path/name instead.
for pattern in \
  "gz sim" \
  "ros_gz_bridge/parameter_bridge" \
  "sim_explore.launch.py" \
  "dome_nav/slam_manager_node" \
  "dome_nav/pluggable_explore_manager_node" \
  "lib/robot_state_publisher/robot_state_publisher" \
  "lib/tf2_ros/static_transform_publisher" \
  "async_slam_toolbox_node" \
  "nav2_controller/controller_server" \
  "nav2_smoother/smoother_server" \
  "nav2_planner/planner_server" \
  "nav2_route/route_server" \
  "nav2_behaviors/behavior_server" \
  "nav2_bt_navigator/bt_navigator" \
  "nav2_waypoint_follower/waypoint_follower" \
  "nav2_velocity_smoother/velocity_smoother" \
  "nav2_collision_monitor/collision_monitor" \
  "opennav_docking/opennav_docking" \
  "nav2_lifecycle_manager/lifecycle_manager" \
  "foxglove_bridge"
do
  pkill -9 -f "$pattern" 2>/dev/null || true
done
sleep 2

echo "Launching sim_explore.launch.py with Gazebo GUI (map_name=$MAP_NAME)..."
bl dome_nav sim_explore.launch.py --map_name "$MAP_NAME" &
BL_PID=$!

echo "Waiting 15s for the stack to come up..."
sleep 15

if ! kill -0 "$BL_PID" 2>/dev/null; then
  echo "Launch process (PID $BL_PID) has already exited — check the log under"
  echo "~/.ros/log/ for the failure before trusting the topic check below."
fi

echo "Starting foxglove_bridge (ws://localhost:8765, use_sim_time:=true)..."
ros2 launch foxglove_bridge foxglove_bridge_launch.xml use_sim_time:=true \
  > /tmp/foxglove_bridge.log 2>&1 &
FOXGLOVE_PID=$!

cleanup() {
  echo "Stopping foxglove_bridge..."
  kill -9 "$FOXGLOVE_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2
if ! kill -0 "$FOXGLOVE_PID" 2>/dev/null; then
  echo "foxglove_bridge failed to start — check /tmp/foxglove_bridge.log"
fi

echo "Checking required topics (publisher present, not just topic-list membership)..."
MISSING=0
for topic in /map /scan /odom /clock /explore/status; do
  pub_count="$(ros2 topic info "$topic" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')"
  if [ -n "$pub_count" ] && [ "$pub_count" -gt 0 ]; then
    echo "  OK      $topic (publishers: $pub_count)"
  else
    echo "  MISSING $topic"
    MISSING=1
  fi
done

if [ "$MISSING" -eq 0 ]; then
  echo "All required topics present."
else
  echo "Some topics missing — check launch output above."
fi

echo "Stack is running with Gazebo GUI (PID $BL_PID)."
echo "foxglove_bridge is running (PID $FOXGLOVE_PID) — connect Foxglove Studio to ws://localhost:8765"
echo "Ctrl-C to stop both."
wait $BL_PID
