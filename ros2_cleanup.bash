#!/bin/bash

PACKAGE_NAME="ros2_snc_9" # Change this to your package name

if [ -z "$1" ]; then
    WS_DIR=~/ros2_ws  # Default Workspace directory
else
    WS_DIR=$1
fi

SRC_DIR="$WS_DIR/src/$PACKAGE_NAME"

# Check if workspace exists
if [ ! -d "$WS_DIR" ]; then
    echo "Error: Workspace '$WS_DIR' not found. Did you forget to pass the workspace as an argument?"
    exit 1
fi

cd $WS_DIR

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
