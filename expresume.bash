#!/usr/bin/env bash
# expresume.bash - publish the exploration_resume intent to a running dome_nav
# stack (resumes after a paused-on-failure NAV abort).
ros2 topic pub --once /intent std_msgs/msg/String \
  'data: "{\"name\": \"exploration_resume\", \"source\": \"cli\", \"slots\": {}}"'
