# AI Line Detection

ROS2 package for AI-based line detection using YOLOv8 segmentation with 3-class detection and color-coded overlay visualization.

## Overview

This package provides a ROS2 node that:
- Subscribes to camera image topics (`/camera/image_raw`)
- Detects 3 types of road markings using YOLOv8 segmentation model:
  - **Class 0 (lane)**: Pink overlay - general lane areas
  - **Class 1 (line)**: Green overlay - lane boundary lines
  - **Class 2 (stop_line)**: Blue overlay - stop lines
- Calculates cross-track error (XTE) for lane following
- Publishes processed images and error values
- Optional OpenCV window for real-time visualization

## Features

- **YOLOv8 3-Class Segmentation**: Detects lane, line, and stop_line simultaneously
- **Color-Coded Overlays**: Pink (lane), Green (line), Blue (stop_line)
- **Lane Spacing Learning**: Dynamically learns lane width from 2-line detections
- **Virtual Lane Generation**: Creates virtual second line when only 1 detected using learned spacing
- **Line Merging**: Intelligently merges lines closer than 150px (handles wide lines with gaps)
- **Real-time Processing**: Optimized for Jetson devices (~50-80ms per frame)
- **Cross-Track Error**: Calculates deviation from lane center
- **FPS Monitoring**: Displays processing speed
- **OpenCV Window**: Optional real-time visualization window
- **Multiple Outputs**: Publishes overlay image and XTE value

## Topics

### Subscribed Topics
- `/camera/ai_lane_detect` (sensor_msgs/Image): Cropped ROI image from CSI camera (640x100)
  - Alternative: `/camera/image_raw` for full resolution

### Published Topics
- `/ai_line_detection/overlay_image` (sensor_msgs/Image): Original image with color-coded overlays (pink/green/blue)
- `/xte/vision` (std_msgs/Float32): Cross-track error in pixels

## Installation

### 1. Install Dependencies

```bash
# Install YOLOv8
pip3 install ultralytics

# Install OpenCV (if not already installed)
sudo apt install python3-opencv

# Install ROS2 CV Bridge
sudo apt install ros-humble-cv-bridge
```

### 2. Place YOLOv8 Model

Place your trained YOLOv8 segmentation model in the `weights/` directory:

```bash
# Example: Copy your trained model
cp /path/to/your/best.pt ~/2025_aa10_ros2_ws/src/AI/ai_line_detection/weights/
```

**Model Requirements:**
- YOLOv8 segmentation model (`.pt` file)
- Class 0 should be "lane"
- Trained on line/lane detection dataset

### 3. Build the Package

```bash
cd ~/2025_aa10_ros2_ws
colcon build --packages-select ai_line_detection --symlink-install
source install/setup.bash
```

## Usage

### Quick Start - Test with Images

```bash
# Launch both image publisher and AI line detection
ros2 launch ai_line_detection test_with_images.launch.py
```

### Production - With Real Camera

```bash
# Terminal 1: Launch CSI Camera
ros2 launch jetson_csi_camera csi_camera.launch.py

# Terminal 2: Launch AI Line Detection
ros2 run ai_line_detection ai_line_detection_node
```

### Launch with Custom Parameters

```bash
# Test with images
ros2 launch ai_line_detection test_with_images.launch.py \
    publish_rate:=5.0 \
    conf_threshold:=0.4 \
    show_window:=True

# Production
ros2 launch ai_line_detection ai_line_detection.launch.py \
    model_path:=/path/to/your/model.pt \
    conf_threshold:=0.5 \
    camera_topic:=/camera/ai_lane_detect \
    overlay_alpha:=0.6
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | string | `weights/best.pt` | Path to YOLOv8 model |
| `conf_threshold` | float | `0.3` | Confidence threshold (0.0-1.0) |
| `camera_topic` | string | `/camera/ai_lane_detect` | Camera topic to subscribe |
| `overlay_alpha` | float | `0.5` | Overlay transparency (0.0-1.0) |
| `show_window` | bool | `True` | Show OpenCV window |

## Complete System Example

### Terminal 1: Launch CSI Camera
```bash
ros2 launch jetson_csi_camera csi_camera.launch.py
```

### Terminal 2: Launch AI Line Detection
```bash
ros2 launch ai_line_detection ai_line_detection.launch.py
```

### Terminal 3: View Overlay Image
```bash
# Using rqt_image_view
rqt_image_view /ai_line_detection/overlay_image

# Or using ros2 run
ros2 run rqt_image_view rqt_image_view /ai_line_detection/overlay_image
```

### Terminal 4: Monitor Cross-Track Error
```bash
ros2 topic echo /xte/vision
```

## Integration with Car Control

This package publishes cross-track error to `/xte/vision`, which can be used by `aa10_car_yaw_control`:

```bash
# Terminal 1: CSI Camera
ros2 launch jetson_csi_camera csi_camera.launch.py

# Terminal 2: AI Line Detection
ros2 launch ai_line_detection ai_line_detection.launch.py

# Terminal 3: Powerpack Driver
ros2 launch amap_powerpack_single_driver powerpack_driver.launch.py

# Terminal 4: Yaw Control (Vision mode)
ros2 launch aa10_car_yaw_control aa10_car_yaw_control.launch.py

# Terminal 5: Set to Vision Control Mode
ros2 topic pub --once /car_control/steering_control_mode std_msgs/msg/Int8 "{data: 1}"
```

## Visualization

The overlay image displays:
- **Yellow highlighted lines**: Detected lane lines with transparency
- **FPS**: Processing frames per second
- **XTE**: Cross-track error in pixels

## Troubleshooting

### CUDA Out of Memory Error

**Symptoms**: `CUDA error: out of memory` or `CUBLAS_STATUS_ALLOC_FAILED`

**Cause**: Background Python processes occupying GPU memory

**Solution**:
```bash
# Kill all Python processes
sudo killall -9 python3
sleep 2
nvidia-smi  # Verify GPU is free

# Or reboot
sudo reboot
```

**Prevention**:
- Always use `Ctrl+C` to stop nodes properly
- Run only one AI process at a time
- Avoid background execution (`&`)

### Model Not Found
```bash
# Check if model exists
ls ~/2025_aa10_ros2_ws/src/AI/ai_line_detection/weights/best.pt

# If not, copy your trained model there
cp /path/to/your/best.pt ~/2025_aa10_ros2_ws/src/AI/ai_line_detection/weights/
```

### No Detection
- Lower confidence threshold: `conf_threshold:=0.2`
- Check if camera is publishing: `ros2 topic echo /camera/ai_lane_detect`
- Verify model is trained for 3 classes (lane, line, stop_line)

### OpenCV Window Not Showing
- Window opens automatically with waitKey + delay
- If using `ai_line_detection_node` alone, need camera node running
- Check `show_window:=True` parameter

### Poor Performance
- Use cropped ROI image (`/camera/ai_lane_detect` 640x100) instead of full resolution
- Increase confidence threshold to filter false positives
- Use smaller YOLOv8 model (e.g., YOLOv8n instead of YOLOv8m)

### Camera Topic Not Found
```bash
# List available topics
ros2 topic list | grep camera

# Change camera topic parameter
ros2 run ai_line_detection ai_line_detection_node --ros-args -p camera_topic:=/camera/image_raw
```

## Algorithm

1. **Image Input**: Receive BGR image from CSI camera
2. **YOLOv8 Inference**: Run segmentation model on image
3. **Mask Creation**: Extract binary mask for lane class (class 0)
4. **Overlay Generation**: Apply yellow color to mask with transparency
5. **XTE Calculation**: Find lane centroid and calculate error from image center
6. **Visualization**: Add FPS and XTE text to overlay image
7. **Publishing**: Publish overlay image, mask, and XTE value

## Color Customization

To change the overlay color, edit [ai_line_detection_node.py](ai_line_detection/ai_line_detection_node.py#L147):

```python
# Current: Yellow (B, G, R)
overlay_image = self.overlay_mask_on_image(
    cv_image,
    lane_mask,
    color=(0, 255, 255),  # Yellow in BGR
    alpha=self.overlay_alpha
)

# For Green: (0, 255, 0)
# For Red: (0, 0, 255)
# For Cyan: (255, 255, 0)
# For Magenta: (255, 0, 255)
```

## Model Training

This package expects a YOLOv8 segmentation model with:
- **Class 0**: lane
- **Task**: Segmentation (not detection)

Train your model using:
```bash
yolo segment train data=your_data.yaml model=yolov8n-seg.pt epochs=100
```

## License

MIT

## Author

aMAP Team
