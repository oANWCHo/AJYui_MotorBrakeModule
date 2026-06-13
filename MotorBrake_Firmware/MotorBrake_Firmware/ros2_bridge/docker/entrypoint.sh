#!/bin/bash
# Source ROS 2 + the MotorBrake overlay, then exec whatever command was passed.
set -e
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
