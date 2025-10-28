#!/bin/bash
# AI Line Detection Test Script
# Launches both image publisher and AI line detection nodes

echo "========================================="
echo "AI Line Detection Test"
echo "========================================="

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 environment not sourced!"
    echo "Run: source /home/amap/2025_aa10_ros2_ws/install/setup.bash"
    exit 1
fi

# Go to workspace
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash

echo ""
echo "Launching AI Line Detection with test images..."
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch
ros2 launch ai_line_detection test_with_images.launch.py
