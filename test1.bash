#!/usr/bin/env bash
# test1.bash — Start Gazebo with simple_room world and spawn dome2 robot.
# No nav stack. Used to verify lidar ray visualization in the Gazebo window.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="$SCRIPT_DIR/worlds/simple_room.world"
URDF="$SCRIPT_DIR/config/dome3_sim.urdf"
export LIBGL_ALWAYS_SOFTWARE=1

source /opt/ros/jazzy/setup.bash
source /home/pitosalas/ros2_ws/install/setup.bash

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

echo "Robot spawned. Gazebo is running (PID $GZ_PID)."
echo "In Gazebo: click the lidar sensor in the left panel to enable ray visualization."
wait $GZ_PID
