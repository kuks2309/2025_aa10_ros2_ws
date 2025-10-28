# Front Obstacle Detect

전방 장애물 감지 ROS2 패키지입니다. LiDAR 데이터를 사용하여 ROI(Region of Interest) 내의 장애물을 감지하고 최소 거리를 publish합니다.

## Features

- LaserScan 및 PointCloud2 메시지 지원
- JSON 파일로 ROI 설정 가능
- 장애물 검출 거리 실시간 publish
- 장애물 검출 여부 boolean topic publish

## Configuration

ROI 및 검출 파라미터는 `config/front_obstacle_detect_roi.json` 파일에서 설정합니다:

```json
{
  "roi": {
    "x_min": 0.0,    // ROI X축(전방) 최소값 (m)
    "x_max": 0.8,    // ROI X축(전방) 최대값 (m) - 80cm
    "y_min": -0.3,   // ROI Y축(좌우) 최소값 (m) - 좌측 30cm
    "y_max": 0.3     // ROI Y축(좌우) 최대값 (m) - 우측 30cm
  },
  "detection": {
    "min_points_threshold": 5,     // 최소 포인트 개수
    "distance_threshold": 0.05,    // 거리 임계값 (m)
    "publish_rate_hz": 10.0        // 발행 주기 (Hz)
  },
  "topics": {
    "lidar_input": "/scan",             // 입력 LiDAR 토픽
    "distance_output": "/obstacle/distance",   // 거리 출력 토픽
    "detected_output": "/obstacle/detected"    // 검출 여부 출력 토픽
  }
}
```

## Topics

### Subscribed Topics

- `/scan` (sensor_msgs/LaserScan): LiDAR 스캔 데이터

### Published Topics

- `/obstacle/distance` (std_msgs/Float32): ROI 내 최소 장애물 거리 (m)
- `/obstacle/detected` (std_msgs/Bool): 장애물 검출 여부
- `/obstacle/roi_marker` (visualization_msgs/Marker): ROI 시각화 마커 (RViz2용)

## Build

```bash
cd ~/2025_aa10_ros2_ws
colcon build --packages-select front_obstacle_detect
source install/setup.bash
```

## Run

```bash
ros2 launch front_obstacle_detect front_obstacle_detect.launch.py
```

## Parameters

- `config_file`: ROI 설정 JSON 파일 경로 (필수)

## Example Usage

### 1. 기본 실행
```bash
ros2 launch front_obstacle_detect front_obstacle_detect.launch.py
```

### 2. 커스텀 config 파일 사용
```bash
ros2 run front_obstacle_detect front_obstacle_detect_node --ros-args -p config_file:=/path/to/your/config.json
```

### 3. 토픽 확인
```bash
# 거리 확인
ros2 topic echo /obstacle/distance

# 검출 여부 확인
ros2 topic echo /obstacle/detected
```

## ROI 좌표계 (2D LiDAR)

- **X축**: 전방 방향 (0m ~ 0.8m)
- **Y축**: 좌우 방향 (왼쪽: -, 오른쪽: +)

기본 설정은 차량 전방 80cm, 좌우 30cm 영역을 감지합니다.

RViz2에서 ROI가 **4각형 외곽선**으로 표시되며:
- **초록색**: 장애물 없음
- **빨간색**: 장애물 검출됨
