# AI Line Detection - Test Image Publisher

테스트 이미지를 ROS2 topic으로 발행하여 AI Line Detection을 테스트하는 노드입니다.

## 노드 구성

### 1. image_publisher_node
- **기능**: test_image/images 폴더의 이미지를 순차적으로 ROS2 topic으로 발행
- **Topic**: `/camera/image_raw` (sensor_msgs/Image)
- **Parameters**:
  - `image_folder`: 이미지 폴더 경로 (기본값: 'test_image/images')
  - `publish_rate`: 발행 주기 Hz (기본값: 10.0)
  - `loop`: 이미지 반복 재생 (기본값: True)
  - `camera_topic`: 카메라 토픽 이름 (기본값: '/camera/image_raw')

### 2. ai_line_detection_node
- **기능**: 카메라 이미지에서 AI 기반 차선 검출
- **Subscribe**: `/camera/image_raw`
- **Publish**:
  - `/ai_line_detection/overlay_image`: 검출 결과 오버레이 이미지
  - `/ai_line_detection/mask_image`: 라인 마스크 이미지 (디버그용)
  - `/xte/vision`: Cross-track error (Float32)

## 실행 방법

### ⚠️ 중요: workspace를 빌드하고 source 해야 합니다!

```bash
cd /home/amap/2025_aa10_ros2_ws
colcon build --packages-select ai_line_detection --symlink-install
source install/setup.bash
```

### 방법 1: Launch 파일 사용 (권장)

두 노드를 동시에 실행:
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 launch ai_line_detection test_with_images.launch.py
```

파라미터 설정:
```bash
ros2 launch ai_line_detection test_with_images.launch.py \
  publish_rate:=5.0 \
  loop:=True \
  conf_threshold:=0.3 \
  overlay_alpha:=0.5 \
  show_window:=True
```

OpenCV 윈도우 없이 실행 (토픽만 발행):
```bash
ros2 launch ai_line_detection test_with_images.launch.py show_window:=False
```

### 방법 2: 개별 노드 실행 (2개 터미널 필요)

**⚠️ 중요: OpenCV 윈도우를 보려면 두 노드를 모두 실행해야 합니다!**

Terminal 1 - 이미지 발행 (필수!):
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run ai_line_detection image_publisher_node
```

Terminal 2 - AI 라인 검출:
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run ai_line_detection ai_line_detection_node
```

**주의**: `ai_line_detection_node`만 단독 실행하면 OpenCV 윈도우는 열리지만 빈 화면입니다. 이미지가 `/camera/image_raw` 토픽으로 발행되어야 검출 결과가 표시됩니다.

### 편리한 alias 설정 (선택사항)

`~/.bashrc`에 추가:
```bash
alias ai_line='cd /home/amap/2025_aa10_ros2_ws && source install/setup.bash && ros2 run ai_line_detection ai_line_detection_node'
alias ai_pub='cd /home/amap/2025_aa10_ros2_ws && source install/setup.bash && ros2 run ai_line_detection image_publisher_node'
alias ai_test='cd /home/amap/2025_aa10_ros2_ws && source install/setup.bash && ros2 launch ai_line_detection test_with_images.launch.py'
```

그 후:
```bash
source ~/.bashrc
ai_test  # Launch 파일 실행
```

## 노드 종료

### Ctrl+C로 종료
노드를 종료할 때는 `Ctrl+C`를 누르면 됩니다.

**최신 빌드 후에는 종료 에러가 발생하지 않습니다!**

종료 로직이 개선되어 다음과 같이 깔끔하게 종료됩니다:
- OpenCV 윈도우 자동 닫기
- 노드 리소스 정리
- RCL context 체크 후 종료

만약 이전 빌드를 사용 중이라면:
```bash
cd /home/amap/2025_aa10_ros2_ws
rm -rf build/ai_line_detection install/ai_line_detection
colcon build --packages-select ai_line_detection --symlink-install
source install/setup.bash
```

### 강제 종료 (필요시)
노드가 응답하지 않을 경우:
```bash
# 모든 ROS2 노드 종료
pkill -9 -f "ros2 run"

# 또는 Python 프로세스 종료
pkill -9 python3
```

## 결과 확인

### OpenCV 윈도우 (기본)
노드 실행 시 자동으로 OpenCV 윈도우가 열립니다:
- **윈도우 이름**: "AI Line Detection - Overlay"
- **크기**: 1280x200 (자동 조정 가능)
- **내용**:
  - 분홍색: lane (차선)
  - 녹색: line (라인 마킹)
  - 파란색: stop_line (정지선)
  - 노란색 수직선: 차선 중앙
  - 회색 십자선: 이미지 중심
  - FPS 및 XTE 정보 표시

**단축키**:
- 윈도우 크기 조절 가능 (마우스 드래그)
- OpenCV 윈도우 닫기: 윈도우 X 버튼 (노드는 계속 실행됨)

### RViz2로 시각화
```bash
rviz2
```

다음 토픽 추가:
- `/camera/image_raw`: 원본 이미지
- `/ai_line_detection/overlay_image`: 검출 결과
- `/ai_line_detection/mask_image`: 라인 마스크

### rqt_image_view로 확인
```bash
rqt_image_view
```

### Topic 확인
```bash
# 이미지 발행 확인
ros2 topic echo /camera/image_raw --once

# XTE 값 확인
ros2 topic echo /xte/vision

# Topic 목록
ros2 topic list

# Topic Hz 확인
ros2 topic hz /camera/image_raw
ros2 topic hz /ai_line_detection/overlay_image
```

## 파라미터 조정

### 발행 속도 조정
```bash
ros2 param set /image_publisher publish_rate 2.0
```

### Confidence threshold 조정
```bash
ros2 param set /ai_line_detection conf_threshold 0.4
```

### Overlay 투명도 조정
```bash
ros2 param set /ai_line_detection overlay_alpha 0.7
```

## 검출 결과

### 색상 코드
- **분홍색 (203, 192, 255)**: Class 0 - lane (차선)
- **녹색 (0, 255, 0)**: Class 1 - line (라인 마킹)
- **파란색 (255, 0, 0)**: Class 2 - stop_line (정지선)
- **노란색 (0, 255, 255)**: 차선 중앙선 (계산된 값)

### 기능
- ✅ 2개 라인 검출 시 차선 중앙 계산 및 lane spacing 학습
- ✅ 1개 라인만 검출 시 학습된 spacing으로 가상 라인 생성
- ✅ 넓은 라인 자동 병합 (150px 미만 간격)
- ✅ XTE (Cross Track Error) 계산 및 발행

## 주의사항

1. GPU 메모리가 부족하면 CUDA 오류가 발생할 수 있습니다.
2. 이미지 발행 속도를 너무 높이면 AI 노드가 따라가지 못할 수 있습니다.
3. test_image/images 폴더에 373개의 테스트 이미지가 있어야 합니다.

## 문제 해결

### 이미지가 발행되지 않음
```bash
# 이미지 파일 확인
ls -la /home/amap/2025_aa10_ros2_ws/src/AI/ai_line_detection/test_image/images/*.jpg | wc -l

# 노드 로그 확인
ros2 node list
ros2 node info /image_publisher
```

### AI 검출이 안됨
```bash
# 모델 파일 확인
ls -la /home/amap/2025_aa10_ros2_ws/install/ai_line_detection/share/ai_line_detection/weights/best.pt

# 노드 로그 확인
ros2 node info /ai_line_detection
```

### GPU 메모리 부족
```bash
# 다른 프로세스 종료
pkill -9 python3

# CUDA 캐시 정리 후 재실행
```
