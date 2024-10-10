#!/bin/bash
# start_robomop_driver.sh
# Script to launch the robomop_driver ROS2 node

# Exit immediately if a command exits with a non-zero status
set -e

# Source ROS2 installation
source /opt/ros/jazzy/setup.bash

# Source your ROS2 workspace
source /home/robomop/robomop_ws/install/setup.bash

# Optional: Export any additional environment variables here
# export MY_VAR=some_value

# Launch the robomop_driver node
ros2 run robomop_driver robomop_driver
