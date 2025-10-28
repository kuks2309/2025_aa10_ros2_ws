# AI Line Detection - Test Scripts Usage Guide

이 문서는 `test_image/images` 폴더의 이미지를 사용하여 AI Line Detection을 테스트하는 방법을 설명합니다.

## 준비사항

### 1. YOLOv8 모델 준비
```bash
# weights 폴더에 학습된 모델 파일이 있는지 확인
ls ~/2025_aa10_ros2_ws/src/AI/ai_line_detection/weights/best.pt
```

### 2. 테스트 이미지 확인
```bash
# test_image/images 폴더에 이미지가 있는지 확인
ls ~/2025_aa10_ros2_ws/src/AI/ai_line_detection/test_image/images/
```

## 제공되는 테스트 스크립트

### 📸 1. 단일 이미지 테스트 (test_single_image.py)

한 장의 이미지를 처리하고 결과를 표시합니다.

**기본 사용법:**
```bash
cd ~/2025_aa10_ros2_ws/src/AI/ai_line_detection

# 특정 이미지 테스트
python3 test_single_image.py test_image/images/crop_image_152727_jpg.rf.5f1eb5ed0fb6fd97bcc00d84d63c9c3f.jpg
```

**옵션과 함께 사용:**
```bash
# Confidence threshold 조정 (기본값: 0.3)
python3 test_single_image.py test_image/images/image1.jpg 0.5

# 중간 과정 표시
python3 test_single_image.py test_image/images/image1.jpg 0.3 --show
```

**출력:**
- 화면에 노란색 라인 오버레이 표시
- `test_image/output/` 폴더에 결과 저장
  - `*_overlay.jpg`: 노란색 오버레이 이미지
  - `*_mask.jpg`: 이진 마스크 이미지
- 터미널에 처리 시간, XTE 값 출력

---

### 📁 2. 배치 이미지 테스트 (test_batch_images.py)

폴더 내의 모든 이미지를 자동으로 처리합니다.

**기본 사용법:**
```bash
cd ~/2025_aa10_ros2_ws/src/AI/ai_line_detection

# test_image/images 폴더의 모든 이미지 처리
python3 test_batch_images.py
```

**커스텀 경로 지정:**
```bash
# 입력 폴더와 출력 폴더 지정
python3 test_batch_images.py /path/to/images /path/to/output

# Confidence threshold 조정
python3 test_batch_images.py test_image/images test_image/output 0.5
```

**출력:**
- 각 이미지에 대한 처리 결과 (overlay, mask)
- 처리 통계 출력:
  - 성공/실패 개수
  - 평균 추론 시간
  - 평균 FPS
  - Cross-Track Error 통계 (평균, 표준편차, 최소/최대)
- `summary_grid.jpg`: 최대 16개 이미지의 요약 그리드

**예시 출력:**
```
======================================================================
BATCH PROCESSING SUMMARY
======================================================================
Total images:          150
Successfully processed: 148
Failed:                2
Lane detected:         142 (95.9%)

Average inference time: 45.2ms
FPS (average):         22.1

Cross-Track Error Statistics:
  Mean XTE:     12.3px
  Std XTE:      18.7px
  Min XTE:     -45.2px
  Max XTE:      67.8px
```

---

### 🎮 3. 인터랙티브 뷰어 (test_interactive_viewer.py)

키보드로 이미지를 탐색하면서 실시간으로 파라미터를 조정할 수 있습니다.

**사용법:**
```bash
cd ~/2025_aa10_ros2_ws/src/AI/ai_line_detection

python3 test_interactive_viewer.py
```

**화면 구성:**
```
┌─────────────┬─────────────┐
│  Original   │   Overlay   │  <- 원본 vs 노란색 오버레이
├─────────────┼─────────────┤
│  Heat Map   │   Result    │  <- 마스크 히트맵 vs 최종 결과
└─────────────┴─────────────┘
        정보 패널
```

**키보드 컨트롤:**
| 키 | 기능 |
|----|------|
| `N` 또는 `→` | 다음 이미지 |
| `P` 또는 `←` | 이전 이미지 |
| `+` 또는 `=` | Confidence threshold 증가 (0.05씩) |
| `-` 또는 `_` | Confidence threshold 감소 (0.05씩) |
| `A` | 투명도 감소 (더 투명하게) |
| `D` | 투명도 증가 (더 불투명하게) |
| `S` | 현재 결과 저장 |
| `Q` 또는 `ESC` | 종료 |

**장점:**
- 실시간으로 파라미터 조정하면서 결과 확인
- 여러 뷰를 동시에 비교
- 최적의 confidence threshold 찾기에 유용

---

## 테스트 결과 분석

### 파일 구조
```
ai_line_detection/
├── test_image/
│   ├── images/              # 입력 이미지들
│   │   ├── image1.jpg
│   │   ├── image2.jpg
│   │   └── ...
│   └── output/              # 출력 결과들
│       ├── image1_overlay.jpg   # 노란색 오버레이
│       ├── image1_mask.jpg      # 이진 마스크
│       ├── image2_overlay.jpg
│       ├── image2_mask.jpg
│       └── summary_grid.jpg     # 요약 그리드
```

### 출력 이미지 설명

1. **`*_overlay.jpg`** (노란색 오버레이)
   - 원본 이미지에 노란색 라인 오버레이
   - 빨간색 세로선: 이미지 중심
   - 파란색 점: 라인 중심점 (centroid)
   - 자주색 선: Cross-Track Error
   - 녹색 텍스트: 처리 시간, XTE 값

2. **`*_mask.jpg`** (이진 마스크)
   - 감지된 라인 영역만 흰색으로 표시
   - 나머지는 검은색
   - 세그멘테이션 결과 확인용

3. **`summary_grid.jpg`** (요약 그리드)
   - 최대 16개 이미지의 오버레이를 4x4 그리드로 표시
   - 전체 결과를 한눈에 확인

---

## 사용 예시

### 시나리오 1: 빠른 단일 이미지 확인
```bash
cd ~/2025_aa10_ros2_ws/src/AI/ai_line_detection

# 첫 번째 이미지 확인
python3 test_single_image.py test_image/images/crop_image_152727_jpg.rf.5f1eb5ed0fb6fd97bcc00d84d63c9c3f.jpg
```

### 시나리오 2: 전체 데이터셋 평가
```bash
# 모든 이미지 처리 및 통계 확인
python3 test_batch_images.py

# 결과 확인
ls test_image/output/
eog test_image/output/summary_grid.jpg  # 이미지 뷰어로 열기
```

### 시나리오 3: 최적 파라미터 찾기
```bash
# 인터랙티브 뷰어로 여러 이미지 확인하면서
# +/- 키로 confidence threshold 조정
python3 test_interactive_viewer.py

# 최적값 발견 후 배치 테스트로 전체 확인
python3 test_batch_images.py test_image/images test_image/output 0.45
```

### 시나리오 4: 다른 이미지 폴더 테스트
```bash
# 새로운 이미지 세트 테스트
python3 test_batch_images.py /path/to/new/images /path/to/new/output 0.3
```

---

## 파라미터 설명

### Confidence Threshold (conf_threshold)
- **범위**: 0.0 ~ 1.0
- **기본값**: 0.3
- **의미**: 이 값보다 높은 confidence를 가진 검출만 사용
- **조정 팁**:
  - 값이 높을수록: 확실한 것만 검출 (False Positive 감소)
  - 값이 낮을수록: 더 많이 검출 (Recall 증가, 노이즈도 증가)
  - 일반적으로 0.3 ~ 0.5 사이가 적절

### Overlay Alpha (overlay_alpha)
- **범위**: 0.0 ~ 1.0
- **기본값**: 0.5
- **의미**: 오버레이 투명도
- **조정**:
  - 0.0: 완전 투명 (원본만 보임)
  - 0.5: 반투명 (기본값)
  - 1.0: 완전 불투명 (노란색만 보임)

---

## 문제 해결

### 문제 1: "Model file not found"
```bash
# 모델 파일 확인
ls weights/best.pt

# 없으면 모델 파일 복사
cp /path/to/your/best.pt weights/
```

### 문제 2: "No images found"
```bash
# 이미지 폴더 확인
ls test_image/images/

# 올바른 경로 지정
python3 test_batch_images.py test_image/images test_image/output
```

### 문제 3: 아무것도 검출되지 않음
- Confidence threshold를 낮춰보기: `0.2` 또는 `0.1`
- 모델이 해당 타입의 라인으로 학습되었는지 확인
- 인터랙티브 뷰어로 실시간 확인:
  ```bash
  python3 test_interactive_viewer.py
  # '-' 키를 여러 번 눌러서 threshold 낮추기
  ```

### 문제 4: 너무 많은 False Positive
- Confidence threshold를 높이기: `0.5` 또는 `0.6`
- 배치 테스트로 확인:
  ```bash
  python3 test_batch_images.py test_image/images test_image/output 0.6
  ```

---

## 성능 벤치마크

### Jetson Orin Nano 예상 성능
- **해상도**: 640x480
- **모델**: YOLOv8n-seg
- **예상 FPS**: 15-25 FPS
- **추론 시간**: 40-60ms

### 성능 확인
```bash
# 배치 테스트로 평균 성능 측정
python3 test_batch_images.py
# 출력에서 "Average inference time"과 "FPS (average)" 확인
```

---

## 추가 팁

### 1. 결과 이미지를 동영상으로 만들기
```bash
cd test_image/output
ffmpeg -framerate 10 -pattern_type glob -i '*_overlay.jpg' -c:v libx264 -pix_fmt yuv420p result.mp4
```

### 2. 특정 패턴의 이미지만 처리
```bash
# 특정 번호 범위의 이미지만 복사해서 테스트
mkdir test_subset
cp test_image/images/crop_image_1527{2,3,4}*.jpg test_subset/
python3 test_batch_images.py test_subset output_subset
```

### 3. XTE 값을 CSV로 저장하기
배치 테스트 출력을 파일로 저장:
```bash
python3 test_batch_images.py 2>&1 | tee results.txt
# results.txt에서 XTE 값 추출
```

---

## 다음 단계

테스트가 만족스러우면 ROS2 노드로 실행:

```bash
# 실시간 카메라로 테스트
ros2 launch jetson_csi_camera csi_camera.launch.py
ros2 launch ai_line_detection ai_line_detection.launch.py

# 결과 확인
rqt_image_view /ai_line_detection/overlay_image
```

---

## 요약

| 스크립트 | 용도 | 실행 시간 |
|---------|------|----------|
| `test_single_image.py` | 단일 이미지 빠른 확인 | 1초 미만 |
| `test_batch_images.py` | 전체 데이터셋 평가 | 이미지 수에 비례 |
| `test_interactive_viewer.py` | 파라미터 튜닝 및 탐색 | 사용자 제어 |

**추천 워크플로우:**
1. `test_single_image.py`로 빠른 확인
2. `test_interactive_viewer.py`로 최적 파라미터 찾기
3. `test_batch_images.py`로 전체 성능 평가
4. ROS2 노드로 실시간 테스트
