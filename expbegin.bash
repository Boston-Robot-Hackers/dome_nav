#!/usr/bin/env bash
# expbegin.bash - publish the exploration_start intent to a running dome_nav stack.
ros2 topic pub --once /intent std_msgs/msg/String \
  'data: "{\"name\": \"exploration_start\", \"source\": \"cli\", \"slots\": {}}"'
