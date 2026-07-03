#!/usr/bin/env bash
# test5.bash — Gazebo (GUI) + robot spawn + ros_gz_bridge + RViz2. No slam_toolbox,
# no Nav2, no explore node — just enough ROS2 plumbing (TF, /scan, /odom, /clock)
# for the simulated robot to be visible and drivable in RViz2.
#
# In RViz2: set Fixed Frame to "odom", then add displays for RobotModel, LaserScan
# (topic /scan), and TF.
#
# Usage: ./test5.bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="$SCRIPT_DIR/worlds/simple_room.world"
URDF="$SCRIPT_DIR/config/dome3_sim.urdf"
export LIBGL_ALWAYS_SOFTWARE=1

source /opt/ros/jazzy/setup.bash
source /home/pitosalas/ros2_ws/install/setup.bash

echo "Killing any stale processes from a previous run..."
for pattern in \
  "gz sim" \
  "ros_gz_bridge/parameter_bridge" \
  "lib/robot_state_publisher/robot_state_publisher" \
  "lib/tf2_ros/static_transform_publisher" \
  "rviz2"
do
  pkill -9 -f "$pattern" 2>/dev/null || true
done
sleep 2

echo "Starting Gazebo..."
gz sim -r "$WORLD" &
GZ_PID=$!

echo "Waiting for Gazebo to be ready..."
sleep 6

echo "Spawning dome2 robot..."
ros2 run ros_gz_sim create \
  -world simple_room \
  -name dome2 \
  -x -1.0 -y -1.0 -z 0.05 \
  -string "$(cat "$URDF")"

echo "Starting ros_gz_bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /odom@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance \
  /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /model/dome2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model \
  --ros-args -r /model/dome2/joint_state:=/joint_states &
BRIDGE_PID=$!

echo "Starting robot_state_publisher..."
# robot_description must go through a params file, not -p key:=value on the CLI —
# the URDF is multi-line XML with quotes/angle-brackets that rcl's argument parser
# cannot handle inline (it aborts with RCLInvalidROSArgsError).
RSP_PARAMS_FILE="$(mktemp --suffix=.yaml)"
python3 -c "
import yaml
with open('$URDF') as f:
    urdf = f.read()
data = {'/**': {'ros__parameters': {'robot_description': urdf, 'use_sim_time': True}}}
with open('$RSP_PARAMS_FILE', 'w') as f:
    yaml.safe_dump(data, f)
"
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file "$RSP_PARAMS_FILE" &
RSP_PID=$!

echo "Starting gz_laser_frame_bridge static transform..."
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 laser dome2/base_footprint/lidar \
  --ros-args -p use_sim_time:=true -r __node:=gz_laser_frame_bridge &
STF_PID=$!

echo "Waiting 3s before starting RViz2..."
sleep 3

echo "Starting RViz2..."
ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true &
RVIZ_PID=$!

cleanup() {
  echo "Stopping bridge, robot_state_publisher, static transform, rviz2..."
  kill -9 "$BRIDGE_PID" "$RSP_PID" "$STF_PID" "$RVIZ_PID" 2>/dev/null || true
  rm -f "$RSP_PARAMS_FILE"
}
trap cleanup EXIT

echo "Stack is running. Gazebo PID $GZ_PID. Ctrl-C to stop everything."
wait $GZ_PID
