#!/bin/bash

echo "🔄 Resetting map (Hector SLAM style)..."
echo ""

# Check if SLAM Toolbox is running
if ! ros2 service list | grep -q "/slam_toolbox/clear_changes"; then
    echo "❌ SLAM Toolbox is not running!"
    echo "   Start SLAM first: ros2 launch slam_toolbox_config slam_launch.py slam_mode:=scan_matching"
    exit 1
fi

echo "📡 Calling SLAM Toolbox clear_changes service..."

# Call the clear changes service
if ros2 service call /slam_toolbox/clear_changes slam_toolbox/srv/Clear; then
    echo ""
    echo "✅ Map reset successful!"
    echo "   - Current mapping data cleared"
    echo "   - Robot position reset to origin"
    echo "   - Ready for new mapping session"
    echo ""
    echo "🎯 You can now move the robot to start fresh mapping."
else
    echo ""
    echo "❌ Map reset failed!"
    echo "   Please check if SLAM Toolbox is running properly."
    exit 1
fi