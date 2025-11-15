# SLAM Toolbox Configuration for AA10 Robot

This package provides SLAM Toolbox configuration for the AA10 robot, supporting both mapping and localization modes.

## Features

- **Mapping Mode**: Create new maps using SLAM Toolbox async mapping
- **Localization Mode**: Localize robot position using existing maps
- **Scan Matching Mode**: Real-time localization without prior map (like Hector SLAM)
- **Simulation Support**: Configured for both Gazebo simulation and real hardware
- **Optimized Parameters**: Tuned for the AA10 robot's characteristics

## Package Structure

```
slam_toolbox_config/
├── launch/
│   ├── slam_launch.py              # Main launch file (mapping/localization)
│   ├── online_async_launch.py      # Mapping mode launch
│   └── localization_launch.py      # Localization mode launch
├── config/
│   ├── mapper_params_online_async.yaml    # Mapping parameters
│   └── mapper_params_localization.yaml    # Localization parameters
└── maps/
    └── (your saved maps will go here)
```

## Usage

### 1. Scan Matching Mode (Real-time localization without map - like Hector SLAM)

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch asw_robot_gazebo_sim gazebo_rviz_display.launch.py

# Terminal 2: Launch lidar
ros2 launch sc_mini_ros2 start.launch.py

# Terminal 3: Launch scan matching localization
ros2 launch slam_toolbox_config slam_launch.py slam_mode:=scan_matching use_sim_time:=true

# Terminal 4: Control robot (localization works automatically)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_gazebo
```

### 2. Mapping Mode (Creating a new map)

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch asw_robot_gazebo_sim gazebo_rviz_display.launch.py

# Terminal 2: Launch lidar
ros2 launch sc_mini_ros2 start.launch.py

# Terminal 3: Launch SLAM mapping
ros2 launch slam_toolbox_config slam_launch.py slam_mode:=mapping use_sim_time:=true

# Terminal 4: Control robot to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_gazebo
```

### 3. Localization Mode (Using existing map)

```bash
# First save your map from mapping mode:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: 
  data: 'my_map'"

# Then launch localization:
ros2 launch slam_toolbox_config slam_launch.py slam_mode:=localization use_sim_time:=true map:=/path/to/your/map.yaml
```

### 3. Real Hardware Usage

```bash
# For real robot (use_sim_time:=false)
ros2 launch slam_toolbox_config slam_launch.py slam_mode:=mapping use_sim_time:=false
```

## SLAM Toolbox Services

### Available Services

- `/slam_toolbox/save_map` - Save current map
- `/slam_toolbox/serialize_map` - Serialize map for later use
- `/slam_toolbox/deserialize_map` - Load serialized map
- `/slam_toolbox/toggle_interactive_mode` - Toggle interactive mode
- `/slam_toolbox/clear_changes` - Clear recent changes

### Saving Maps

```bash
# Save map as image files (png + yaml)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: 
  data: 'my_map'"

# Save map in native SLAM Toolbox format
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: 
  data: 'my_map.posegraph'"
```

## Parameters

### Key Parameters (can be tuned in config files):

- `resolution`: Map resolution (default: 0.05m)
- `max_laser_range`: Maximum laser range (default: 20.0m)
- `minimum_travel_distance`: Minimum distance to travel before adding scan (default: 0.5m)
- `loop_search_maximum_distance`: Maximum distance for loop closure (default: 3.0m)
- `use_scan_matching`: Enable scan matching (default: true)
- `do_loop_closing`: Enable loop closure (default: true)

## Troubleshooting

### Common Issues:

1. **No map updates**: Check if `/scan` topic is publishing data
2. **Transform errors**: Ensure `base_footprint` frame exists
3. **Poor mapping quality**: Adjust `minimum_travel_distance` and scan matching parameters
4. **Loop closure issues**: Tune `loop_search_maximum_distance` and loop closure parameters

### Debugging Commands:

```bash
# Check scan topic
ros2 topic echo /scan --once

# Check transforms
ros2 run tf2_tools view_frames

# Monitor SLAM Toolbox status
ros2 topic echo /slam_toolbox/scan_visualization
```

## Integration with AA10 Wall Following

The SLAM system works alongside the existing wall following system:

```bash
# Run SLAM + Wall Following
ros2 launch slam_toolbox_config slam_launch.py slam_mode:=mapping &
ros2 launch aa10_car_yaw_control aa10_car_yaw_control.launch.py &
python3 /path/to/wall_following_gui.py
```

## Dependencies

- `slam_toolbox`
- `nav2_map_server`
- `nav2_lifecycle_manager`
- `tf2_ros`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`