# Jetson CSI Camera ROS2 Package

ROS2 driver for CSI cameras on NVIDIA Jetson Orin Nano.

## Features

- Native CSI camera support using GStreamer
- Hardware-accelerated video processing (NVMM)
- Configurable resolution and framerate
- Image flip support
- Publishes `sensor_msgs/Image` and `sensor_msgs/CameraInfo`

## Requirements

- NVIDIA Jetson Orin Nano
- CSI camera (e.g., Raspberry Pi Camera Module v2)
- ROS2 Humble
- OpenCV with GStreamer support
- NVIDIA JetPack SDK

## Installation

```bash
cd ~/2025_aa10_ros2_ws
colcon build --packages-select jetson_csi_camera
source install/setup.bash
```

## Usage

### 1. Standalone Image Capture Tool (Non-ROS2)

Simple C++ application for capturing and saving images from CSI camera:

```bash
# Source the workspace
source install/setup.bash

# Run the capture tool
ros2 run jetson_csi_camera csi_capture_image_save

# Or run directly
./install/jetson_csi_camera/lib/jetson_csi_camera/csi_capture_image_save
```

**Controls:**
- **SPACE**: Save current frame (images saved to `./images/YYYY-MM-DD/image_HHMMSS.jpg`)
- **ESC**: Exit program

**Features:**
- GStreamer-based CSI camera access
- 1280x720 resolution @ 30 FPS
- On-screen display with detection area overlay
- Pure original frames saved (no overlay in saved images)
- Automatic date-based folder organization

### 2. ROS2 Node (Continuous Streaming)

#### Basic Launch

```bash
ros2 launch jetson_csi_camera csi_camera.launch.py
```

### Custom Parameters

```bash
ros2 launch jetson_csi_camera csi_camera.launch.py \
    camera_id:=0 \
    image_width:=1920 \
    image_height:=1080 \
    framerate:=30 \
    flip_method:=2
```

### Image Scaling (0.5x)

```bash
# 1280x720 captured, 640x360 published (50% size)
ros2 launch jetson_csi_camera csi_camera.launch.py image_scale:=0.5

# 60fps + 0.5x scaling for AI processing
ros2 launch jetson_csi_camera csi_camera.launch.py framerate:=60 image_scale:=0.5
```

### Run Node Directly

```bash
ros2 run jetson_csi_camera csi_camera_node \
    --ros-args \
    -p camera_id:=0 \
    -p image_width:=1280 \
    -p image_height:=720
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_id` | int | 0 | CSI camera sensor ID (0 or 1) |
| `image_width` | int | 1280 | Image width in pixels |
| `image_height` | int | 720 | Image height in pixels |
| `framerate` | int | 30 | Camera framerate (fps) |
| `flip_method` | int | 0 | Flip method (0=none, 2=rotate-180, etc.) |
| `camera_frame_id` | string | "camera_link" | TF frame ID |
| `image_scale` | double | 1.0 | **Image scaling factor (0.5 = 50%, 1.0 = 100%)** |
| `publish_rate` | double | 30.0 | Publishing rate (Hz) |

## Topics

### Published Topics

- `/camera/image_raw` (sensor_msgs/Image) - Raw camera images
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Camera calibration info

## Flip Methods

- 0: No flip
- 1: Counterclockwise 90 degrees
- 2: Rotate 180 degrees
- 3: Clockwise 90 degrees
- 4: Horizontal flip
- 5: Upper right diagonal
- 6: Vertical flip
- 7: Upper left diagonal

## Troubleshooting

### Camera not detected

```bash
# Check if camera is detected
ls /dev/video*

# Test camera with gst-launch
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! nvoverlaysink
```

### Permission issues

```bash
sudo usermod -aG video $USER
# Then logout and login again
```

### Check GStreamer plugins

```bash
gst-inspect-1.0 nvarguscamerasrc
```

## License

Apache-2.0
