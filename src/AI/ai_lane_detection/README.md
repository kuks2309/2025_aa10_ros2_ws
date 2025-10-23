# AI_Lane_detection

ROS2 package for AI-based lane detection with center error calculation for Gazebo simulation.

## Overview

This package provides a ROS2 node that:
- Subscribes to Gazebo camera image topics
- Detects lane lines using computer vision techniques
- Calculates the error between image center and lane center
- Publishes the error as geometry_msgs/Point
- Publishes processed images with lane detection visualization

## Features

- Real-time lane detection using traditional computer vision (Canny edge detection + Hough transform)
- Lane center error calculation (pixels from image center)
- FPS monitoring and performance tracking
- Visual overlay with detected lanes and error information
- Robust handling of single or double lane detection

## Topics

### Subscribed Topics
- `/camera/image_raw` (sensor_msgs/Image): Input camera feed from Gazebo

### Published Topics
- `/lane_center_error` (geometry_msgs/Point): Lane center error (x: error in pixels, y: 0, z: 0)
- `/lane_detection/processed_image` (sensor_msgs/Image): Processed image with lane detection overlay

## Usage

1. Build the package:
```bash
colcon build --packages-select AI_Lane_detection
```

2. Source the workspace:
```bash
source install/setup.bash
```

3. Run the lane detection node:
```bash
ros2 run AI_Lane_detection lane_detection_node
```

## Parameters

The lane detection algorithm uses the following parameters:
- Canny edge detection: low_threshold=50, high_threshold=150
- Gaussian blur: kernel_size=(5,5)
- Hough line detection: threshold=20, minLineLength=20, maxLineGap=300
- Lane slope thresholds: left_lane < -0.3, right_lane > 0.3

## Algorithm

1. **Image Preprocessing**: Convert to grayscale and apply Gaussian blur
2. **Edge Detection**: Use Canny edge detector to find edges
3. **Region of Interest**: Focus on bottom half of image where lanes are expected
4. **Line Detection**: Use Hough transform to detect line segments
5. **Lane Classification**: Classify lines as left/right lanes based on slope
6. **Lane Averaging**: Average multiple line segments into representative lanes
7. **Center Calculation**: Calculate lane center and compute error from image center

## Error Calculation

The error is calculated as:
```
error_x = lane_center_x - image_center_x
```

- Positive error: Lane center is to the right of image center
- Negative error: Lane center is to the left of image center
- Zero error: Lane center aligns with image center

## Integration with dss_yolo_realtime.py

This package is inspired by the structure of `dss_yolo_realtime.py` but adapted for ROS2:
- Similar image processing pipeline
- Comparable performance monitoring
- Real-time processing capabilities
- Error calculation and visualization features

## Dependencies

- rclpy
- sensor_msgs
- geometry_msgs
- cv_bridge
- opencv-python
- numpy