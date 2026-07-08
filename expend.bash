#!/usr/bin/env bash
# expend.bash - publish the exploration_stop intent to a running dome_nav stack.
ros2 topic pub --once /intent std_msgs/msg/String \
  'data: "{\"name\": \"exploration_stop\", \"source\": \"cli\", \"slots\": {}}"'
