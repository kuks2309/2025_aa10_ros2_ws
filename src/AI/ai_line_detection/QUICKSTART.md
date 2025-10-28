# AI Line Detection - Quick Start Guide

## 🚀 빠른 시작

### 1단계: Workspace 빌드 (최초 1회 또는 코드 수정 후)

```bash
cd /home/amap/2025_aa10_ros2_ws
rm -rf build/ai_line_detection install/ai_line_detection  # 클린 빌드
colcon build --packages-select ai_line_detection --symlink-install
source install/setup.bash
```

### 2단계: 실행

**⚠️ 중요사항**
- **토픽**: `/camera/ai_lane_detect` (640x100 크롭 이미지)
- **OpenCV 윈도우**: 자동으로 표시됩니다 (클릭 불필요)
- **GPU 메모리**: 실행 전 다른 Python 프로세스 종료 필수!

**방법 A: 테스트 이미지로 실행 (권장)**
```bash
# Launch 파일 사용 - image_publisher + ai_line_detection 동시 실행
ros2 launch ai_line_detection test_with_images.launch.py
```

**방법 B: 실제 카메라로 실행**

Terminal 1 - CSI 카메라:
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 launch jetson_csi_camera csi_camera.launch.py
```

Terminal 2 - AI 라인 검출:
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run ai_line_detection ai_line_detection_node
```

**방법 C: 테스트 이미지 - 개별 노드 실행**

Terminal 1 - 이미지 발행:
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

### 3단계: 결과 확인

- **OpenCV 윈도우**: 자동으로 열리며 실시간 검출 결과 표시
- **터미널 로그**: FPS, XTE 값 확인

### 종료

`Ctrl+C` - 깔끔하게 종료됩니다 (에러 없음)

---

## ⚡ 한 줄 명령어

```bash
cd /home/amap/2025_aa10_ros2_ws && source install/setup.bash && ros2 launch ai_line_detection test_with_images.launch.py
```

## 🎨 OpenCV 윈도우 내용

- **분홍색**: lane (차선)
- **녹색**: line (라인 마킹)
- **파란색**: stop_line (정지선)
- **노란색 수직선**: 차선 중앙
- **회색 십자선**: 이미지 중심

## 📊 주요 파라미터

```bash
# 이미지 발행 속도 조절 (기본: 5Hz)
ros2 launch ai_line_detection test_with_images.launch.py publish_rate:=10.0

# OpenCV 윈도우 끄기 (헤드리스 환경)
ros2 launch ai_line_detection test_with_images.launch.py show_window:=False

# Confidence threshold 조절 (기본: 0.3)
ros2 launch ai_line_detection test_with_images.launch.py conf_threshold:=0.4

# 카메라 토픽 변경 (기본: /camera/ai_lane_detect)
ros2 run ai_line_detection ai_line_detection_node --ros-args -p camera_topic:=/camera/image_raw
```

## 🔧 문제 해결

### 모델 파일을 찾을 수 없음
```bash
ls -la /home/amap/2025_aa10_ros2_ws/src/AI/ai_line_detection/weights/best.pt
```
파일이 없으면 모델을 다시 학습하거나 복사하세요.

### 이미지를 찾을 수 없음
```bash
ls /home/amap/2025_aa10_ros2_ws/src/AI/ai_line_detection/test_image/images/*.jpg | wc -l
```
373개의 이미지가 있어야 합니다.

### GPU 메모리 부족 (CUDA out of memory)

**원인**: 백그라운드 Python 프로세스가 GPU 메모리를 점유

**해결**:
```bash
# 방법 1: 모든 Python 프로세스 종료
sudo killall -9 python3
sleep 2
nvidia-smi  # GPU 상태 확인

# 방법 2: 특정 프로세스만 종료
pkill -9 -f test_image_viewer
pkill -9 -f ai_line_detection_node

# 방법 3: 시스템 재부팅 (가장 확실)
sudo reboot
```

**예방**:
- 사용 후 항상 `Ctrl+C`로 종료
- 한 번에 하나의 AI 프로세스만 실행
- 백그라운드 실행(`&`) 금지

### 종료 시 에러 발생
최신 빌드로 업데이트:
```bash
cd /home/amap/2025_aa10_ros2_ws
rm -rf build/ai_line_detection install/ai_line_detection
colcon build --packages-select ai_line_detection --symlink-install
source install/setup.bash
```

## 📝 더 자세한 정보

- [README_TEST.md](README_TEST.md) - 전체 사용 가이드
- [README.md](README.md) - 패키지 정보

## ✅ 체크리스트

- [ ] Workspace 빌드 완료
- [ ] `source install/setup.bash` 실행
- [ ] 모델 파일 존재 확인
- [ ] 테스트 이미지 373개 확인
- [ ] 노드 실행
- [ ] OpenCV 윈도우 확인
- [ ] 종료 테스트

모든 항목이 체크되면 정상 작동합니다! 🎉
