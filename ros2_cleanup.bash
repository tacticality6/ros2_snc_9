#!/bin/bash

PACKAGE_NAME="ROS2_SNC_9" # Change this to your package name
WS_DIR=~/ros2_ws  # Change this if your workspace is in a different location
SRC_DIR="$WS_DIR/src/$PACKAGE_NAME"

# Check if the package exists
if [ ! -d "$SRC_DIR" ]; then
    echo "Error: Package '$PACKAGE_NAME' not found in $WS_DIR/src/"
    exit 1
fi

#change to workspace directory
cd $WS_DIR
# Remove the package
echo "Removing package '$PACKAGE_NAME'..."
rm -rf "$SRC_DIR"

# Clean build, install, and log directories
echo "Cleaning up build, install, and log directories..."
rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"

echo "Package '$PACKAGE_NAME' removed successfully."
