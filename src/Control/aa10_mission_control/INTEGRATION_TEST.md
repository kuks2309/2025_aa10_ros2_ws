# aa10_mission_control - AI Line Detection 통합 테스트 가이드

## 미션 시나리오

### Mission 0: 출발 대기
- **상태**: 정지
- **설명**: 출발 가림막 제거 대기
- **동작**:
  - 차량 정지 (속도 = 0)
  - Lane control 비활성화
  - Control mode = STOP
  - Obstacle detect 노드로부터 전방 장애물 거리 모니터링
- **전환 조건**:
  - `obstacle_detected_ == false` (장애물 없음) **또는**
  - `obstacle_distance_ > 0.5m` (가림막이 50cm 이상 멀어짐)

### Mission 1: AI 라인 검출 주행
- **상태**: 주행 중
- **설명**: AI 라인 검출을 이용한 자율 주행
- **동작**:
  - Lane control 활성화
  - Control mode = LANE_CONTROL (Vision control)
  - 주행 속도 = 150
- **전환 조건**: 정지선 검출 시 (Y > 80px) Mission 2로 전환

### Mission 2: 정지선 정지
- **상태**: 정지
- **설명**: 정지선에서 완전 정지
- **동작**:
  - 차량 정지 (속도 = 0)
  - Lane control 비활성화
  - Control mode = STOP
- **전환 조건**: 미션 종료

## 토픽 흐름

```
┌─────────────────────┐       ┌─────────────────────────┐
│  CSI Camera         │       │  LiDAR (2D)             │
└──────────┬──────────┘       └──────────┬──────────────┘
           │ /camera/ai_lane_detect      │ /scan
           v                              v
┌─────────────────────────────┐  ┌──────────────────────────┐
│  ai_line_detection_node     │  │ front_obstacle_detect    │
└──────────┬──────────────────┘  └──────────┬───────────────┘
           │                                 │
           ├─> /xte/vision ──────────────┐  ├─> /obstacle/distance ─┐
           │                              │  │                        │
           └─> /stop_line_position ───┐  │  └─> /obstacle/detected ─┤
                                       │  │                           │
                                       v  v                           v
                                ┌────────────────────────────────────────┐
                                │     aa10_mission_control               │
                                │  - Mission 0: Wait (barrier detect)    │
                                │  - Mission 1: Drive (lane following)   │
                                │  - Mission 2: Stop (stop line)         │
                                └──────────┬─────────────────────────────┘
                                           │
                                           ├─> /car_control/steering_control_mode
                                           ├─> /car_control/speed
                                           └─> /flag/lane_control_set
                                                    │
                                                    v
                                        ┌─────────────────────────┐
                                        │ aa10_car_yaw_control    │
                                        │  - Vision mode          │
                                        │  - Uses /xte/vision     │
                                        └─────────────────────────┘
```

## 통합 테스트 절차

### 준비사항

```bash
# GPU 메모리 정리 (중요!)
sudo killall -9 python3
sleep 2
nvidia-smi  # "No running processes found" 확인

# Workspace source
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
```

### 방법 1: 테스트 이미지 사용

**Terminal 1 - AI Line Detection (이미지 발행 포함):**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 launch ai_line_detection test_with_images.launch.py
```

**Terminal 2 - Mission Control:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run aa10_mission_control aa10_mission_control_node
```

**Terminal 3 - Run Flag 발행 (시작 신호):**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 topic pub --once /flag/race_run std_msgs/msg/Bool "{data: true}"
```

**Terminal 4 - 토픽 모니터링:**
```bash
# XTE 확인
ros2 topic echo /xte/vision

# Stop line 확인
ros2 topic echo /stop_line_position

# Control mode 확인
ros2 topic echo /car_control/steering_control_mode

# Speed 확인
ros2 topic echo /car_control/speed
```

### 방법 2: 실제 카메라 사용

**Terminal 1 - CSI Camera:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 launch jetson_csi_camera csi_camera.launch.py
```

**Terminal 2 - AI Line Detection:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run ai_line_detection ai_line_detection_node
```

**Terminal 3 - Mission Control:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run aa10_mission_control aa10_mission_control_node
```

**Terminal 4 - Run Flag:**
```bash
ros2 topic pub --once /flag/race_run std_msgs/msg/Bool "{data: true}"
```

## 예상 로그 출력

### Mission Control 로그:

```
[INFO] [aa10_mission_control]: Mission 0: Waiting for start (remove barrier)
[INFO] [aa10_mission_control]: Mission: 0 | IMU: 0.0° | Speed: 0.00 | Mode: Real
[INFO] [aa10_mission_control]: Barrier removed - Starting mission!
[INFO] [aa10_mission_control]: Mission 1: Lane following with AI line detection
[INFO] [aa10_mission_control]: Mission: 1 | IMU: 0.0° | Speed: 150.00 | Mode: Real
[INFO] [aa10_mission_control]: Stop line detected close (y=95.0) - Stopping!
[INFO] [aa10_mission_control]: Mission 2: Stopped at stop line
[INFO] [aa10_mission_control]: Mission: 2 | IMU: 0.0° | Speed: 0.00 | Mode: Real
```

### AI Line Detection 로그:

```
[INFO] [ai_line_detection_node]: YOLOv8 model loaded successfully!
[INFO] [ai_line_detection_node]: OpenCV window enabled
[INFO] [ai_line_detection_node]: AI Line Detection Node initialized
[INFO] [ai_line_detection_node]: Processing time: 78.5ms, XTE: -12.0px
[INFO] [ai_line_detection_node]: Stop line detected at y=95.0px
[INFO] [ai_line_detection_node]: Processing time: 82.1ms, XTE: -8.5px
```

## 정지선 거리 임계값 조정

현재 정지선 임계값: **Y > 80px** (640x100 이미지 기준)

임계값을 조정하려면 [aa10_mission_control_node.cpp](src/aa10_mission_control_node.cpp#L254) 수정:

```cpp
// 더 일찍 정지 (더 먼 거리에서)
if (stop_line_position_ > 60.0) {  // 원래: 80.0

// 더 늦게 정지 (더 가까워지면)
if (stop_line_position_ > 90.0) {  // 원래: 80.0
```

## 가림막 감지 로직 수정

현재는 5초 후 자동 시작하는 임시 로직입니다. 실제 구현 시:

### 옵션 1: 수동 신호
```bash
# Mission 0에서 대기 중일 때
ros2 topic pub --once /mission/start_signal std_msgs/msg/Bool "{data: true}"
```

### 옵션 2: LiDAR 감지
```cpp
case 0:
    // 가림막 감지: LiDAR 전방 거리 > 1.0m
    if (lidar_front_obstacle_distance_ > 1.0) {
        mission_flag_ = 1;
        RCLCPP_INFO(this->get_logger(), "Barrier removed - Starting mission!");
    }
    break;
```

### 옵션 3: 카메라 감지
```cpp
case 0:
    // 카메라로 라인 검출 시작
    if (stop_line_position_ == 0.0 && xte_vision_ != 0.0) {
        // 정지선은 없고 라인은 검출됨 = 가림막 제거됨
        mission_flag_ = 1;
    }
    break;
```

## 파라미터 튜닝

### 주행 속도 조정
[aa10_mission_control_node.cpp](src/aa10_mission_control_node.cpp#L248):
```cpp
publishSpeedReal(150);  // 기본값
// 더 빠르게: 200
// 더 느리게: 100
```

### 정지선 검출 임계값
[aa10_mission_control_node.cpp](src/aa10_mission_control_node.cpp#L254):
```cpp
if (stop_line_position_ > 80.0) {  // 기본값
// 이미지 크기: 640x100
// Y=100: 최하단 (매우 가까움)
// Y=80: 하단 부근 (가까움)
// Y=50: 중간 (보통)
// Y=20: 상단 부근 (멀리)
```

## 문제 해결

### Mission이 0에서 진행되지 않음
```bash
# run_flag 확인
ros2 topic echo /flag/race_run

# run_flag가 false이면 발행
ros2 topic pub --once /flag/race_run std_msgs/msg/Bool "{data: true}"
```

### Stop line이 검출되지 않음
```bash
# AI line detection 오버레이 확인
rqt_image_view /ai_line_detection/overlay_image
# 파란색 영역이 정지선 (class 2)

# Stop line position 토픽 확인
ros2 topic echo /stop_line_position
# 아무것도 출력되지 않으면 정지선이 검출되지 않는 것
```

### Control mode가 변경되지 않음
```bash
# aa10_car_yaw_control이 실행 중인지 확인
ros2 node list | grep yaw_control

# Control mode 토픽 확인
ros2 topic echo /car_control/steering_control_mode
# 2 = LANE_CONTROL (Vision mode)
# 0 = STOP
```

### CUDA out of memory
```bash
# GPU 메모리 정리
sudo killall -9 python3
sleep 2
nvidia-smi

# 또는 시스템 재부팅
sudo reboot
```

## 다음 단계

이 기본 미션이 성공적으로 작동하면:

1. **더 복잡한 미션 추가**:
   - 차선 변경
   - 장애물 회피
   - 원 주행

2. **안전 기능 추가**:
   - LiDAR 긴급 정지
   - Watchdog 타이머
   - Failsafe 모드

3. **성능 최적화**:
   - PID 제어 튜닝
   - 속도 프로파일 최적화
   - 정지선 검출 정확도 향상

## 요약

✅ **완료된 통합:**
- AI line detection → Mission control
- Stop line detection → Auto stop
- 3단계 미션 (대기 → 주행 → 정지)

📋 **테스트 체크리스트:**
- [ ] GPU 메모리 정리 확인
- [ ] 모든 노드 실행 확인
- [ ] run_flag 발행 확인
- [ ] Mission 0 → 1 자동 전환 확인
- [ ] Lane following 동작 확인
- [ ] Stop line 검출 확인
- [ ] Mission 1 → 2 자동 전환 확인
- [ ] 최종 정지 확인
