#!/bin/bash

set -e

# setup ros1 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the local workspace
source "/ros_ws/install/setup.bash"

# Run the launch file
ros2 launch vicon_receiver client.launch.py