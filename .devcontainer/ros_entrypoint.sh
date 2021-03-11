#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# setup cv_bridge
source "/ws/vision_opencv/install/setup.bash"

exec "$@"