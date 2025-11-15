# Waypoint Control Package

ROS2 패키지로 SLAM을 사용하여 현재 위치에서 목적지까지 자율 주행을 수행합니다.

## 기능

- **Pure Pursuit 알고리즘**: 경로 추종을 위한 Pure Pursuit 제어 알고리즘 사용
- **토픽 기반 경로 생성**: `/current_pose`와 `/goal_pose` 토픽을 통해 자동으로 경로 생성
- **파일 기반 Waypoint**: 수동으로 waypoint 파일 로드 가능
- **실시간 제어**: 조향각과 속도를 실시간으로 계산하여 발행

## 노드 구성

### 1. waypoint_follower (C++)
- **구독 토픽**:
  - `/waypoint_path` (nav_msgs/Path): 추종할 경로
  - `/odom` (nav_msgs/Odometry): 로봇의 현재 위치

- **발행 토픽**:
  - `/sc_mini/sc_mini/cmd_vel` (geometry_msgs/Twist): 속도 명령

- **파라미터**:
  - `lookahead_distance`: Pure Pursuit lookahead 거리 (기본값: 0.5m)
  - `max_linear_velocity`: 최대 선속도 (기본값: 0.3 m/s)
  - `min_linear_velocity`: 최소 선속도 (기본값: 0.1 m/s)
  - `max_angular_velocity`: 최대 각속도 (기본값: 1.0 rad/s)
  - `goal_tolerance`: 목표 도달 허용 오차 (기본값: 0.15m)
  - `wheelbase`: 차량 축간 거리 (기본값: 0.2m)

### 2. waypoint_publisher (Python)
- **구독 토픽**:
  - `/current_pose` (geometry_msgs/PoseStamped): 현재 위치
  - `/goal_pose` (geometry_msgs/PoseStamped): 목표 위치
  - `/odom` (nav_msgs/Odometry): Odometry (current_pose 대안)

- **발행 토픽**:
  - `/waypoint_path` (nav_msgs/Path): 생성된 경로

- **파라미터**:
  - `num_intermediate_points`: 중간 경유점 개수 (기본값: 10)
  - `use_straight_line`: 직선 경로 사용 여부 (기본값: true)

## 빌드

```bash
cd ~/2025_aa10_ros2_ws
colcon build --packages-select waypoint_control
source install/setup.bash
```

## 사용 방법

### 방법 1: 토픽을 통한 자동 경로 생성

1. 패키지 실행:
```bash
ros2 launch waypoint_control waypoint_control.launch.py
```

2. 현재 위치 발행 (예시):
```bash
ros2 topic pub --once /current_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

3. 목표 위치 발행:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 2.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

### 방법 2: Waypoint 파일 사용

1. Waypoint 파일 생성 (format: x y theta):
```
# example_waypoints.txt
0.0 0.0 0.0
1.0 0.0 0.0
1.0 1.0 1.57
0.0 1.0 3.14
```

2. 파일과 함께 실행:
```bash
ros2 run waypoint_control waypoint_publisher.py /path/to/waypoints.txt
ros2 run waypoint_control waypoint_follower --ros-args --params-file $(ros2 pkg prefix waypoint_control)/share/waypoint_control/config/waypoint_control.yaml
```

### 방법 3: 개별 노드 실행

```bash
# Waypoint follower
ros2 run waypoint_control waypoint_follower

# Waypoint publisher
ros2 run waypoint_control waypoint_publisher.py
```

## 파라미터 조정

`config/waypoint_control.yaml` 파일을 수정하여 제어 파라미터를 조정할 수 있습니다:

```yaml
waypoint_follower:
  ros__parameters:
    lookahead_distance: 0.5
    max_linear_velocity: 0.3
    max_angular_velocity: 1.0
    goal_tolerance: 0.15
```

## 테스트

경로가 정상적으로 생성되고 발행되는지 확인:
```bash
ros2 topic echo /waypoint_path
```

속도 명령이 발행되는지 확인:
```bash
ros2 topic echo /sc_mini/sc_mini/cmd_vel
```

## 주의사항

- SLAM 또는 Localization이 실행 중이어야 `/odom` 토픽이 제공됩니다
- 목표 위치는 `map` 프레임 기준으로 제공되어야 합니다
- 로봇의 wheelbase 파라미터를 실제 로봇에 맞게 조정하세요

## 토픽 인터페이스

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/current_pose` | geometry_msgs/PoseStamped | 구독 | 현재 로봇 위치 |
| `/goal_pose` | geometry_msgs/PoseStamped | 구독 | 목표 위치 |
| `/odom` | nav_msgs/Odometry | 구독 | Odometry 정보 |
| `/waypoint_path` | nav_msgs/Path | 발행 | 생성된 경로 |
| `/sc_mini/sc_mini/cmd_vel` | geometry_msgs/Twist | 발행 | 속도 명령 |
