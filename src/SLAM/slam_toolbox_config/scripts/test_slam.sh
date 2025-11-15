#!/bin/bash

echo "=== SLAM Toolbox Configuration Test ==="
echo ""

# Source the workspace
source /home/amap/aMAP_ROS2_AA10_Simulator/install/setup.bash

echo "1. Checking package installation..."
if ros2 pkg list | grep -q "slam_toolbox_config"; then
    echo "✓ slam_toolbox_config package found"
else
    echo "✗ slam_toolbox_config package not found"
    exit 1
fi

echo ""
echo "2. Checking SLAM Toolbox installation..."
if ros2 pkg list | grep -q "slam_toolbox"; then
    echo "✓ slam_toolbox package found"
else
    echo "✗ slam_toolbox package not found - please install with:"
    echo "  sudo apt install ros-humble-slam-toolbox"
    exit 1
fi

echo ""
echo "3. Checking launch files..."
launch_dir="/home/amap/aMAP_ROS2_AA10_Simulator/install/slam_toolbox_config/share/slam_toolbox_config/launch"
if [ -f "$launch_dir/slam_launch.py" ]; then
    echo "✓ Main launch file found"
else
    echo "✗ Launch files not found"
    exit 1
fi

echo ""
echo "4. Checking config files..."
config_dir="/home/amap/aMAP_ROS2_AA10_Simulator/install/slam_toolbox_config/share/slam_toolbox_config/config"
if [ -f "$config_dir/mapper_params_online_async.yaml" ]; then
    echo "✓ Configuration files found"
else
    echo "✗ Configuration files not found"
    exit 1
fi

echo ""
echo "5. Testing launch file syntax..."
if ros2 launch slam_toolbox_config slam_launch.py --show-args > /dev/null 2>&1; then
    echo "✓ Launch file syntax is valid"
else
    echo "✗ Launch file syntax error"
    exit 1
fi

echo ""
echo "=== All tests passed! ==="
echo ""
echo "Usage examples:"
echo "# Mapping mode:"
echo "ros2 launch slam_toolbox_config slam_launch.py slam_mode:=mapping use_sim_time:=true"
echo ""
echo "# Localization mode:"
echo "ros2 launch slam_toolbox_config slam_launch.py slam_mode:=localization use_sim_time:=true"
echo ""
echo "See README.md for detailed usage instructions."