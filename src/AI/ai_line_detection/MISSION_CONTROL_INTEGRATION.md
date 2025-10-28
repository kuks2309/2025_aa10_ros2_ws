# AI Line Detection - Mission Control 통합 가이드

## 토픽 연결

### ai_line_detection_node가 발행하는 토픽

| 토픽 | 타입 | 설명 | 연결 대상 |
|------|------|------|-----------|
| `/xte/vision` | std_msgs/Float32 | Cross-track error (차선 중앙으로부터 이탈 거리, 픽셀) | aa10_car_yaw_control |
| `/stop_line_position` | std_msgs/Float32 | 정지선 Y 좌표 (이미지 하단에 가까울수록 큰 값) | aa10_mission_control |
| `/lane_status` | std_msgs/String | 차선 상태 "LEFT,CENTER,RIGHT" (현재: "CLEAR,CLEAR,CLEAR") | aa10_mission_control |
| `/ai_line_detection/overlay_image` | sensor_msgs/Image | 검출 결과 오버레이 이미지 | rqt_image_view (디버깅용) |

### aa10_mission_control이 구독하는 토픽

| 토픽 | 타입 | 설명 | 발행 노드 |
|------|------|------|-----------|
| `/stop_line_position` | std_msgs/Float32 | 정지선 위치 | ai_line_detection_node |
| `/lane_status` | std_msgs/String | 차선 장애물 상태 | ai_line_detection_node (추후 장애물 검출 기능 추가 예정) |

## 데이터 흐름

```
┌─────────────────────┐
│  CSI Camera Node    │
└──────────┬──────────┘
           │ /camera/ai_lane_detect (640x100)
           │
           v
┌─────────────────────────────────┐
│  AI Line Detection Node         │
│  - YOLOv8 Segmentation          │
│  - 3 Classes: lane/line/stop    │
└──────────┬──────────────────────┘
           │
           ├─────> /xte/vision ──────────────────┐
           │                                      │
           ├─────> /stop_line_position ─────┐   │
           │                                 │   │
           └─────> /lane_status ───────┐    │   │
                                        │    │   │
                                        v    v   v
                            ┌───────────────────────────┐
                            │  aa10_mission_control     │
                            │  - Mission state machine  │
                            │  - Lane change logic      │
                            └───────────┬───────────────┘
                                        │
                                        v
                            ┌───────────────────────────┐
                            │  aa10_car_yaw_control     │
                            │  - Steering control       │
                            └───────────────────────────┘
```

## 정지선 검출 로직

### detect_stop_line_position()

```python
def detect_stop_line_position(self, stop_line_mask):
    """
    정지선의 Y 좌표를 검출

    Returns:
        float: 정지선의 최하단 Y 좌표 (이미지 하단에 가까울수록 큰 값)
        None: 정지선이 검출되지 않음
    """
```

**동작 방식:**
1. stop_line_mask에서 non-zero 픽셀 찾기
2. Y 좌표의 최댓값 반환 (이미지 하단 = 차량에 가까움)
3. 값이 클수록 정지선이 가까움

**예시:**
- 이미지 크기: 640x100
- 정지선이 하단에 검출: Y = 95 (가까움)
- 정지선이 상단에 검출: Y = 10 (멀리 있음)
- 정지선 없음: None

## 차선 상태 발행

현재는 기본값으로 모든 차선이 CLEAR 상태로 발행됩니다:

```python
lane_status_msg = String()
lane_status_msg.data = "CLEAR,CLEAR,CLEAR"  # LEFT,CENTER,RIGHT
self.lane_status_pub.publish(lane_status_msg)
```

**추후 확장 계획:**
- 장애물 검출 기능 추가 시 실제 차선별 장애물 상태 발행
- 형식: "LEFT_STATUS,CENTER_STATUS,RIGHT_STATUS"
- 가능한 값: "CLEAR" 또는 "BLOCKED"

## 통합 테스트

### 1. 테스트 이미지로 확인

```bash
# Terminal 1: AI Line Detection
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 launch ai_line_detection test_with_images.launch.py

# Terminal 2: 토픽 확인
ros2 topic echo /xte/vision
ros2 topic echo /stop_line_position
ros2 topic echo /lane_status
```

### 2. 실제 카메라로 확인

```bash
# Terminal 1: CSI Camera
ros2 launch jetson_csi_camera csi_camera.launch.py

# Terminal 2: AI Line Detection
ros2 run ai_line_detection ai_line_detection_node

# Terminal 3: Mission Control (if needed)
ros2 run aa10_mission_control aa10_mission_control_node

# Terminal 4: 토픽 확인
ros2 topic list | grep -E "xte|stop_line|lane_status"
```

### 3. 토픽 Hz 확인

```bash
# XTE는 매 프레임마다 발행 (~30Hz 실제 카메라, ~5Hz 테스트)
ros2 topic hz /xte/vision

# 정지선은 검출될 때만 발행
ros2 topic hz /stop_line_position

# 차선 상태는 매 프레임마다 발행
ros2 topic hz /lane_status
```

## 파라미터 설정

ai_line_detection_node의 파라미터를 통해 동작 조정 가능:

```bash
# Confidence threshold 조절 (정지선 검출 민감도)
ros2 run ai_line_detection ai_line_detection_node --ros-args -p conf_threshold:=0.4

# OpenCV 윈도우 비활성화 (헤드리스 모드)
ros2 run ai_line_detection ai_line_detection_node --ros-args -p show_window:=False
```

## 문제 해결

### 정지선이 검출되지 않음

**확인 사항:**
```bash
# 1. 토픽이 발행되는지 확인
ros2 topic list | grep stop_line_position

# 2. 토픽 데이터 확인
ros2 topic echo /stop_line_position

# 3. 오버레이 이미지로 시각적 확인
rqt_image_view /ai_line_detection/overlay_image
# 파란색 영역이 정지선
```

**해결 방법:**
- Confidence threshold 낮추기: `conf_threshold:=0.2`
- 모델 재학습으로 정지선 검출 성능 향상
- 테스트 이미지에 정지선이 포함되어 있는지 확인

### Mission Control이 토픽을 받지 못함

**확인:**
```bash
# 두 노드가 모두 실행 중인지 확인
ros2 node list
# /ai_line_detection_node
# /mission_control_node

# 토픽 연결 확인
ros2 topic info /stop_line_position
# Publishers: ai_line_detection_node
# Subscribers: mission_control_node
```

**해결:**
- 두 노드가 같은 ROS_DOMAIN_ID를 사용하는지 확인
- 네트워크 방화벽 설정 확인 (localhost는 문제없음)

## 요약

✅ **완료된 통합:**
- `/xte/vision` - Cross-track error 발행
- `/stop_line_position` - 정지선 위치 발행
- `/lane_status` - 차선 상태 발행 (기본값)

⏳ **추후 확장:**
- `/lane_status`에 실제 장애물 검출 데이터 반영
- 차선 변경 의사결정 로직 연동
- 정지선 거리 기반 자동 정지 기능

📚 **관련 문서:**
- [QUICKSTART.md](QUICKSTART.md) - 빠른 시작 가이드
- [README.md](README.md) - 패키지 상세 정보
- [HOW_TO_SEE_OPENCV_WINDOW.md](HOW_TO_SEE_OPENCV_WINDOW.md) - OpenCV 윈도우 가이드
