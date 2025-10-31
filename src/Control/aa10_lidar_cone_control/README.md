# AA10 LiDAR Cone Control

ROS2 package for detecting and tracking traffic cones using 2D LiDAR data, with cross-track error calculation for autonomous navigation.

## Overview

This package processes 2D LiDAR point cloud data to:
- Detect traffic cones in a maze/track environment
- Identify and track left/right cone sequences
- Calculate cross-track error (XTE) for path following control

## Installation

This package is located in: `/home/amap/2025_aa10_ros2_ws/src/Control/aa10_lidar_cone_control`

### Dependencies

- ROS2 (Humble or later)
- PCL (Point Cloud Library)
- OpenCV
- cv_bridge
- pcl_ros
- tf2

### Build

```bash
cd /home/amap/2025_aa10_ros2_ws
colcon build --packages-select aa10_lidar_cone_control
source install/setup.bash
```

## Usage

### Launch Node

```bash
ros2 launch aa10_lidar_cone_control lidar_cone_control.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch aa10_lidar_cone_control lidar_cone_control.launch.py \
  lidar2d_cloud_topic:=/your/pointcloud/topic \
  min_blob_area:=350 \
  max_blob_area:=650
```

## Topics

### Subscribed Topics

- `/pointcloud2d` (sensor_msgs/PointCloud2): 2D LiDAR point cloud data
- `handsfree/imu/yaw_degree` (std_msgs/Float32): IMU heading angle in degrees

### Published Topics

- `/xte/lidar_cone` (std_msgs/Float32): Cross-track error calculated from cone positions

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar2d_cloud_topic` | string | `/pointcloud2d` | Input 2D point cloud topic |
| `imu_heading_angle_topic` | string | `handsfree/imu/yaw_degree` | IMU heading topic |
| `min_blob_area` | int | 400 | Minimum blob area for cone detection (pixels²) |
| `max_blob_area` | int | 600 | Maximum blob area for cone detection (pixels²) |
| `image_scale` | int | 400 | Image scale factor (pixels/meter) |
| `cone_spacing_min` | double | 0.17 | Minimum expected cone spacing (meters) |
| `cone_spacing_max` | double | 0.23 | Maximum expected cone spacing (meters) |

## Algorithm

1. **Point Cloud to Image Conversion**: Converts 2D LiDAR points to binary image (400×500 pixels)
2. **Morphological Processing**: Applies dilation to connect nearby points
3. **Blob Detection**: Uses OpenCV connected components for cone detection
4. **Cone Tracking**: Identifies left/right cone sequences using nearest-neighbor search
5. **XTE Calculation**: Computes cross-track error from centerline between cones

## Improvements from ROS1 Version

- **Bug Fixes**:
  - Fixed incorrect array indexing in point cloud access
  - Fixed logic error in cone distance comparison
  - Removed memory leak (unused pointer allocation)

- **Code Quality**:
  - Refactored into C++ class structure
  - Encapsulated state variables as class members
  - Used ROS2 parameter system with declarations
  - Improved logging with RCLCPP macros
  - Configurable cone spacing thresholds

- **ROS2 Migration**:
  - Updated to ROS2 rclcpp API
  - Modern C++ shared pointers for callbacks
  - ROS2 launch file with launch arguments

## Visualization

The node creates two OpenCV windows:
- **Lidar window**: Binary dilated point cloud image
- **result**: Colored visualization with detected cones and tracking lines
  - Yellow: Left cones and path
  - Red: Right cones and path
  - Green: Connection between first left/right cones
  - Purple: Center line reference

## Notes

- Requires X11 display for OpenCV visualization windows
- Expected cone spacing is 17-23 cm (adjustable via parameters)
- Image scale: 400 pixels = 1 meter (adjustable)
- Cone detection works best with consistent cone sizes and regular spacing

## Author

Converted from ROS1 to ROS2 by Claude Code
Original package: aa10_lidar_cone_tracking_control
