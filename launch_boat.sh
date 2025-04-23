#!/bin/bash
# Navigate to your ROS2 workspace source folder (adjust the path as necessary)
cd /home/boat/Desktop/version4/TAFLAB_boatpi_roshumble

# Source your local install/setup.bash to set up the ROS2 environment
source install/setup.bash

sudo pigpiod
# Launch the boat_launch launch file
ros2 launch boat_launch boat_launch.py