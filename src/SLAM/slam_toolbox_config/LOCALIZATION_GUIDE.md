# 맵 없이 위치 인식 가이드 (Scan Matching Localization)

## 🎯 목적
처음 방문하는 환경에서 **사전 맵 없이** 실시간으로 로봇의 위치를 추정합니다.
Hector SLAM과 유사한 방식으로 라이다 스캔 매칭만을 사용합니다.

## 🚀 빠른 시작

### 1. 기본 실행
```bash
# 시뮬레이션 환경에서 실행
ros2 launch slam_toolbox_config slam_launch.py slam_mode:=scan_matching use_sim_time:=true
```

### 2. 맵 리셋 (Hector SLAM 스타일)
```bash
# 현재 맵을 리셋하고 새로 시작
ros2 service call /reset_map std_srvs/srv/Empty
```

### 2. 전체 시스템 실행 (권장)
```bash
# Terminal 1: Gazebo 시뮬레이션
ros2 launch asw_robot_gazebo_sim gazebo_rviz_display.launch.py

# Terminal 2: 라이다
ros2 launch sc_mini_ros2 start.launch.py

# Terminal 3: 스캔 매칭 위치 인식
ros2 launch slam_toolbox_config slam_launch.py slam_mode:=scan_matching

# Terminal 4: 벽 추적과 함께 (선택)
ros2 launch aa10_car_yaw_control aa10_car_yaw_control.launch.py
python3 /path/to/wall_following_gui.py
```

## 📊 실시간 위치 확인

### RViz2에서 확인:
- **map 프레임**: 글로벌 좌표계
- **odom 프레임**: 오도메트리 좌표계  
- **base_footprint**: 로봇 위치
- **Laser Scan**: 현재 라이다 데이터
- **Map**: 실시간 생성되는 맵

### 토픽 모니터링:
```bash
# 현재 위치 (odom → base_footprint)
ros2 run tf2_tools view_frames

# 변환 행렬 확인
ros2 run tf2_ros tf2_echo map base_footprint

# 맵 토픽 확인
ros2 topic echo /map --once
```

## ⚙️ 주요 특징

### Hector SLAM과 유사한 동작:
- ✅ **맵 없이 위치 추정**: 사전 맵이 필요하지 않음
- ✅ **실시간 처리**: 20Hz 업데이트 주기
- ✅ **스캔 매칭**: 라이다 데이터만으로 위치 계산
- ✅ **동시 매핑**: 위치 추정과 동시에 맵 생성
- ✅ **루프 클로저**: 정확도 향상을 위한 루프 감지

### 최적화된 파라미터:
- `minimum_travel_distance: 0.1m` - 매우 민감한 위치 업데이트
- `transform_publish_period: 0.02` - 50Hz TF 발행
- `map_update_interval: 0.5` - 빠른 맵 업데이트
- `correlation_search_space_resolution: 0.005` - 고해상도 매칭

## 🔧 성능 튜닝

### 더 정확한 위치 추정:
```yaml
# config/scan_matching_localization.yaml에서 조정
minimum_travel_distance: 0.05    # 더 민감하게
minimum_travel_heading: 0.05     # 더 민감하게
correlation_search_space_resolution: 0.003  # 더 정밀하게
```

### 더 빠른 처리:
```yaml
transform_publish_period: 0.01   # 100Hz
map_update_interval: 1.0         # 느린 맵 업데이트
```

## 🏗️ 실제 로봇에서 사용

```bash
# use_sim_time:=false로 설정
ros2 launch slam_toolbox_config slam_launch.py slam_mode:=scan_matching use_sim_time:=false
```

## 🚨 주의사항

1. **초기 위치**: 로봇이 시작할 때의 위치가 (0,0)이 됩니다
2. **드리프트**: 장시간 사용 시 위치 오차가 누적될 수 있습니다
3. **환경 요구사항**: 벽이나 장애물이 있는 환경에서 잘 작동합니다
4. **라이다 품질**: 노이즈가 적은 양질의 라이다 데이터가 필요합니다

## 📈 성능 모니터링

```bash
# 변환 빈도 확인
ros2 topic hz /tf

# 맵 업데이트 빈도 확인  
ros2 topic hz /map

# SLAM 상태 확인
ros2 topic echo /slam_toolbox/scan_visualization --once
```

## 🔄 다른 모드와 비교

| 모드 | 사전 맵 | 정확도 | 처리 속도 | 용도 |
|------|---------|--------|-----------|------|
| **scan_matching** | ❌ 불필요 | ⭐⭐⭐ | ⭐⭐⭐⭐ | **실시간 탐색** |
| mapping | ❌ 불필요 | ⭐⭐⭐⭐ | ⭐⭐⭐ | 맵 생성 |
| localization | ✅ 필요 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 정밀 위치 추정 |

**scan_matching 모드**는 Hector SLAM처럼 미지의 환경에서 실시간 위치 인식에 최적화되어 있습니다.