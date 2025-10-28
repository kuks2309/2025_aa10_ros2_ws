# OpenCV 윈도우 표시 가이드

## 윈도우 자동 표시

OpenCV 윈도우는 **자동으로 표시**됩니다:
- `cv2.waitKey(10)` + `time.sleep()` 조합 사용
- 윈도우를 클릭하거나 키를 누를 필요 없음
- 약 100ms 후 자동으로 포커스 획득

## 토픽 정보

`ai_line_detection_node`는 **구독자(Subscriber)** 노드입니다:
- **기본 토픽**: `/camera/ai_lane_detect` (640x100 크롭 이미지)
- **대체 토픽**: `/camera/image_raw` (전체 해상도)
- 이미지가 발행되지 않으면 처리할 데이터가 없어서 빈 화면입니다

## ✅ 해결 방법

### 방법 1: 테스트 이미지 사용 (권장)

**Launch 파일로 두 노드 동시 실행:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 launch ai_line_detection test_with_images.launch.py
```

### 방법 2: 실제 카메라 사용

**Terminal 1 - CSI 카메라:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 launch jetson_csi_camera csi_camera.launch.py
```

**Terminal 2 - AI 라인 검출:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run ai_line_detection ai_line_detection_node
```

### 방법 3: 테스트 이미지 - 개별 실행

**Terminal 1 - 이미지 발행:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run ai_line_detection image_publisher_node
```

**Terminal 2 - AI 라인 검출:**
```bash
cd /home/amap/2025_aa10_ros2_ws
source install/setup.bash
ros2 run ai_line_detection ai_line_detection_node
```

## 노드 구조

### 테스트 이미지 사용 시:
```
[image_publisher_node]  ---> /camera/ai_lane_detect ---> [ai_line_detection_node] ---> OpenCV Window
      (발행자)                      (토픽 640x100)                (구독자)                  (자동 표시)
```

### 실제 카메라 사용 시:
```
[csi_camera_node]  ---> /camera/ai_lane_detect ---> [ai_line_detection_node] ---> OpenCV Window
   (CSI 카메라)              (토픽 640x100)                (구독자)                  (자동 표시)
```

## 확인 방법

### 1. 토픽 발행 확인
```bash
# 새로운 터미널에서
ros2 topic list | grep camera
# /camera/ai_lane_detect 토픽이 있어야 함

ros2 topic hz /camera/ai_lane_detect
# 초당 5개 메시지 발행 확인 (테스트 이미지 기본 설정)
# 또는 30개 (실제 카메라)
```

### 2. 노드 실행 확인
```bash
ros2 node list
# /image_publisher 와 /ai_line_detection 두 노드가 모두 실행 중이어야 함
```

### 3. OpenCV 윈도우 확인
- 윈도우 이름: "AI Line Detection - Overlay"
- 크기: 1280x200
- 내용: 분홍색(lane), 녹색(line), 파란색(stop_line), 노란색 중앙선

## 예상 로그 출력

**image_publisher_node:**
```
[INFO] Found 373 images
[INFO] [1/373] Published: crop_image_152727...
[INFO] [2/373] Published: crop_image_152731...
```

**ai_line_detection_node:**
```
[INFO] YOLOv8 model loaded successfully!
[INFO] OpenCV window enabled
[INFO] Processing time: 49.6ms, XTE: 29.0px
```

## 문제 해결

### OpenCV 윈도우는 열리는데 빈 화면인 경우
→ 이미지를 발행하는 노드가 실행되지 않았습니다:
- 테스트: `image_publisher_node` 실행 필요
- 실제: `csi_camera_node` 실행 필요

### 윈도우가 아예 열리지 않는 경우
→ `show_window` 파라미터가 False로 설정되었습니다:
```bash
ros2 run ai_line_detection ai_line_detection_node --ros-args -p show_window:=True
```

### 윈도우가 클릭해야 보이는 경우
→ 최신 버전으로 업데이트하세요 (waitKey + delay 조합 사용):
```bash
cd /home/amap/2025_aa10_ros2_ws
rm -rf build/ai_line_detection install/ai_line_detection
colcon build --packages-select ai_line_detection --symlink-install
source install/setup.bash
```

### CUDA Out of Memory 에러
→ 백그라운드 프로세스 정리:
```bash
sudo killall -9 python3
sleep 2
nvidia-smi  # GPU 확인
```

### "No images found" 에러
→ 테스트 이미지 폴더 확인:
```bash
ls /home/amap/2025_aa10_ros2_ws/src/AI/ai_line_detection/test_image/images/*.jpg | wc -l
# 373개가 출력되어야 함
```

### "Model file not found" 에러
→ 모델 파일 확인:
```bash
ls -la /home/amap/2025_aa10_ros2_ws/src/AI/ai_line_detection/weights/best.pt
```

## 빠른 테스트

```bash
# 1. Workspace source
cd /home/amap/2025_aa10_ros2_ws && source install/setup.bash

# 2. Launch 실행
ros2 launch ai_line_detection test_with_images.launch.py

# 3. OpenCV 윈도우 확인
# - 1초에 약 10장의 이미지가 처리되어야 함
# - 녹색 라인이 검출되어야 함
# - 노란색 중앙선이 표시되어야 함

# 4. 종료
Ctrl+C
```

완료! 🎉
