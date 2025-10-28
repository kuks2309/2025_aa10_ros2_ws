# Encoder Status Integration - aa10_mission_control

## 개요

`aa10_mission_control` 노드가 차량의 엔코더 데이터를 구독하여 차량의 위치와 속도 정보를 활용할 수 있도록 통합했습니다.

## 토픽 정보

### 구독하는 토픽

| 토픽 | 타입 | 발행 주기 | 발행 노드 | 설명 |
|------|------|-----------|-----------|------|
| `/car_control/encoder_status` | `amap_powerpack_single_driver/msg/EncoderStatus` | 10Hz | powerpack_driver_node | 엔코더 상태 정보 |

### EncoderStatus 메시지 구조

```msg
int32 position_pulse      # 엔코더 위치 (펄스)
int16 speed_pulse         # 엔코더 속도 (펄스/초)
float32 position_mm       # 위치 (밀리미터) - 누적 거리
float32 speed_mms         # 속도 (mm/초)
```

## 사용 가능한 변수

`aa10_mission_control_node`에서 다음 멤버 변수로 접근 가능:

```cpp
double encoder_position_mm_;  // 차량 누적 위치 (밀리미터)
double encoder_speed_mms_;    // 차량 속도 (mm/초)
```

## 활용 예시

### 1. 특정 거리 주행 후 동작

```cpp
case 3:
    // 3m 주행 후 다음 미션으로 전환
    publishLaneControl(true);
    publishControlMode(LANE_CONTROL);
    publishSpeedReal(150);

    if (encoder_position_mm_ > 3000.0) {  // 3000mm = 3m
        mission_flag_ = 4;
        RCLCPP_INFO(this->get_logger(), "Traveled 3 meters - Next mission!");
    }
    break;
```

### 2. 거리 기반 정지

```cpp
case 5:
    // 시작 위치로부터 5m 지점에서 정지
    if (encoder_position_mm_ > 5000.0) {  // 5000mm = 5m
        publishLaneControl(false);
        publishControlMode(STOP);
        publishSpeedReal(0);
        RCLCPP_INFO(this->get_logger(), "Stopped at 5m mark");
        mission_flag_ = 6;
    }
    break;
```

### 3. 속도 모니터링

```cpp
case 7:
    // 속도가 너무 낮으면 경고
    if (encoder_speed_mms_ < 100.0) {  // 100mm/s = 0.1m/s
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Speed too low: %.1f mm/s", encoder_speed_mms_);
    }
    break;
```

### 4. 거리 기반 미션 전환 (출발 후 일정 거리)

```cpp
case 1:
    // AI 라인 검출 주행
    publishLaneControl(true);
    publishControlMode(LANE_CONTROL);
    publishSpeedReal(150);

    // 출발 후 2m 주행했으면 다음 미션
    if (encoder_position_mm_ > 2000.0) {
        mission_flag_ = 2;
        RCLCPP_INFO(this->get_logger(), "2m traveled - switching mission");
    }
    break;
```

## 단위 변환

### 밀리미터 → 미터

```cpp
double position_m = encoder_position_mm_ / 1000.0;
```

### mm/s → m/s

```cpp
double speed_ms = encoder_speed_mms_ / 1000.0;
```

### mm/s → km/h

```cpp
double speed_kmh = (encoder_speed_mms_ / 1000.0) * 3.6;
```

## 주의사항

### 1. 위치 초기화

엔코더 위치는 **누적 거리**입니다:
- powerpack 노드가 시작되면 0부터 시작
- 차량이 후진하면 값이 감소
- 절대 위치가 아닌 상대 위치

### 2. 위치 리셋이 필요한 경우

특정 미션에서 위치를 0으로 리셋하고 싶다면:

**방법 1**: Mission 시작 시점의 위치 저장
```cpp
// 멤버 변수 추가
double mission_start_position_mm_;

// Mission 시작 시
case 5:
    if (mission_flag_ != previous_mission) {
        mission_start_position_mm_ = encoder_position_mm_;
    }

    // 이 미션에서 주행한 거리
    double traveled_in_mission = encoder_position_mm_ - mission_start_position_mm_;

    if (traveled_in_mission > 1000.0) {  // 이 미션에서 1m 주행
        mission_flag_ = 6;
    }
    break;
```

**방법 2**: powerpack 노드 재시작 (권장하지 않음)

### 3. 오버플로우

`position_pulse`는 `int32`이므로 매우 긴 거리 주행 시 오버플로우 가능:
- `position_mm`는 `float32`이므로 실용적 범위 내에서는 문제없음
- 최대값: 약 2,147,483mm = 2,147m (충분함)

## 데이터 흐름

```
┌───────────────────────────┐
│  Powerpack Driver Node    │
│  - Reads encoder values   │
│  - Converts to mm         │
│  - Publishes at 10Hz      │
└────────────┬──────────────┘
             │ /car_control/encoder_status
             │ (10Hz)
             v
┌───────────────────────────┐
│  Mission Control Node     │
│  - Subscribes encoder     │
│  - Uses for mission logic │
│  - Distance-based control │
└───────────────────────────┘
```

## 빌드 및 실행

### 빌드

```bash
cd /home/amap/2025_aa10_ros2_ws
colcon build --packages-select aa10_mission_control --symlink-install
source install/setup.bash
```

### 실행

```bash
# Terminal 1: Powerpack driver (엔코더 발행)
ros2 run amap_powerpack_single_driver powerpack_driver_node

# Terminal 2: Mission control
ros2 run aa10_mission_control aa10_mission_control_node

# Terminal 3: 엔코더 데이터 확인
ros2 topic echo /car_control/encoder_status
```

## 디버깅

### 엔코더 토픽 확인

```bash
# 토픽이 발행되는지 확인
ros2 topic list | grep encoder

# 발행 주기 확인 (10Hz 예상)
ros2 topic hz /car_control/encoder_status

# 데이터 확인
ros2 topic echo /car_control/encoder_status
```

### Mission control 로그 레벨 변경

```bash
# DEBUG 레벨로 실행 (엔코더 데이터 로그 확인)
ros2 run aa10_mission_control aa10_mission_control_node --ros-args --log-level debug
```

DEBUG 모드에서는 다음 로그 출력:
```
[DEBUG] [aa10_mission_control]: Encoder: position=1234.5 mm, speed=567.8 mm/s
```

## Mission 3: 상대 위치 제어 예시

Mission 3에서는 stop line 검출 후 정지한 위치에서 상대 거리로 10cm 더 전진하는 예제를 구현했습니다.

### 동작 흐름

```
Mission 1: AI 라인 검출 주행
    ↓
Stop line 검출 (Y > 40)
    ↓
Mission 2: 정지선에서 정지 (0.5초 대기)
    ↓
Mission 3: 상대 위치로 10cm 전진
    - /car_control/target_position_relative 토픽에 100mm 발행
    - 엔코더로 이동 거리 모니터링
    - 95mm 이상 이동 + 속도 < 5mm/s → 완료
    ↓
Mission 4: 최종 정지
```

### 구현 코드 (Mission 3)

```cpp
case 3:
{
    // 상대 위치 명령 한 번만 전송
    if (!mission3_position_sent_) {
        amap_powerpack_single_driver::msg::TargetPositionRelative pos_msg;
        pos_msg.relative_position = 100.0;  // 100mm = 10cm
        target_position_relative_pub_->publish(pos_msg);

        mission3_start_position_mm_ = encoder_position_mm_;
        mission3_position_sent_ = true;
    }

    // 이동 완료 확인
    double traveled_distance = encoder_position_mm_ - mission3_start_position_mm_;

    if (traveled_distance >= 95.0 && std::abs(encoder_speed_mms_) < 5.0) {
        mission_flag_ = 4;  // 다음 미션으로
        mission3_position_sent_ = false;
    }
    break;
}
```

### 주요 포인트

1. **상대 위치 제어**: 절대 위치가 아닌 현재 위치에서 +100mm 이동
2. **엔코더 피드백**: `encoder_position_mm_`로 실제 이동 거리 확인
3. **정지 확인**: 거리 도달 + 속도 < 5mm/s로 완료 판단
4. **One-shot 명령**: `mission3_position_sent_` 플래그로 중복 전송 방지

## 요약

✅ **완료된 통합:**
- `/car_control/encoder_status` 토픽 구독
- `encoder_position_mm_`, `encoder_speed_mms_` 변수 업데이트
- 미션 로직에서 활용 가능
- 상대 위치 제어 (`/car_control/target_position_relative`) 통합

📊 **활용 가능한 데이터:**
- 누적 주행 거리 (밀리미터)
- 현재 속도 (mm/초)

💡 **활용 예시:**
- 거리 기반 미션 전환
- 정지선까지 거리 계산
- 속도 모니터링 및 제어
- **상대 위치 제어로 정밀한 거리 이동**

## 관련 문서

- [INTEGRATION_TEST.md](INTEGRATION_TEST.md) - AI Line Detection 통합 테스트
- [amap_powerpack_single_driver](../../ROS2_Orin_Nano/amap_powerpack_single_driver/) - Powerpack 드라이버 패키지
