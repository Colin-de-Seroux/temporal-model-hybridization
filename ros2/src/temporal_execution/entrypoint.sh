#!/bin/bash

LOG_DIR="/ros2_ws/.ros/log"
BACKUP_DIR="/ros_logs_backup"

# Function to save logs
function save_logs {
    echo "Saving ROS2 logs to disk..."
    mkdir -p "$BACKUP_DIR"
    cp -r "$LOG_DIR"/* "$BACKUP_DIR"/
    echo "Logs saved to $BACKUP_DIR."
}

# Set trap to call save_logs on exit
trap save_logs EXIT

# Run ROS2 node
echo "Starting ROS2 node..."
source /opt/ros/jazzy/setup.sh
source /ros2_ws/install/setup.sh
ros2 run temporal_execution loop_node > /dev/null 2>&1
