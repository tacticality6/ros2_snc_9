#!/bin/bash

# === CONFIGURATION ===
ROS2_VERSION="humble"  # Change if using another ROS 2 version
WS_DIR=~/ros2_ws        # Adjust workspace path as needed
PACKAGE_NAME="ROS2_SNC_9"



LAUNCH_FILE=$1


cd $WS_DIR

# === SOURCE ROS 2 ENVIRONMENT ===
source /opt/ros/$ROS2_VERSION/setup.bash
source $WS_DIR/install/setup.bash

# === BUILD PACKAGE ===
echo "Building package: $PACKAGE_NAME..."
colcon build --packages-select $PACKAGE_NAME --event-handlers console_direct+

# === SOURCE NEW BUILD ===
source $WS_DIR/install/setup.bash

# === RUN LAUNCH FILE IF PROVIDED ===
if [ -n "$LAUNCH_FILE" ]; then
    echo "Running launch file: $LAUNCH_FILE..."
    ros2 launch $PACKAGE_NAME $LAUNCH_FILE
fi
