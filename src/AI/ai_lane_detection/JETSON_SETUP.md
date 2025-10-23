# Jetson Orin Nano - AI Lane Detection 설치 가이드

이 문서는 Jetson Orin Nano에서 YOLOv8 기반 차선 감지를 GPU로 실행하기 위한 완전한 설치 가이드입니다.

## ✅ 설치 완료 확인

이 가이드대로 설치를 완료하면 다음 환경이 구성됩니다:

```
- PyTorch: 2.3.0 (CUDA 12.4 지원)
- OpenCV: 4.10.0
- NumPy: 1.26.4
- Ultralytics (YOLOv8): 8.3.220
- CUDA: 12.6
- GPU: Orin (Compute Capability 8.7)
```

## 시스템 요구사항

- **하드웨어**: NVIDIA Jetson Orin Nano
- **OS**: Ubuntu (JetPack 6.x, R36)
- **Python**: 3.10
- **CUDA**: 12.x

## 1. 시스템 환경 확인

```bash
# JetPack 버전 확인
cat /etc/nv_tegra_release
# 출력 예: R36 (release), REVISION: 4.7

# CUDA 확인
nvidia-smi

# Python 버전 확인
python3 --version
# 출력: Python 3.10.12
```

## 2. PyTorch 설치 (GPU 지원)

### 2.1 필요한 의존성 설치

```bash
# pip 업그레이드
pip3 install --upgrade pip

# NumPy 1.x 버전 설치 (호환성 위해)
pip3 install "numpy<2"
```

### 2.2 Jetson용 PyTorch 다운로드 및 설치

```bash
# PyTorch 2.3.0 for Jetson (CUDA 12.4) 다운로드
cd /tmp
wget https://nvidia.box.com/shared/static/zvultzsmd4iuheykxy17s4l2n91ylpl8.whl \
  -O torch-2.3.0-cp310-cp310-linux_aarch64.whl

# 설치
pip3 install torch-2.3.0-cp310-cp310-linux_aarch64.whl
```

**중요**: 일반 PyPI의 PyTorch를 설치하지 마세요. Jetson에서는 CUDA를 지원하지 않습니다.

### 2.3 설치 확인

```bash
python3 << 'EOF'
import torch
print('PyTorch Version:', torch.__version__)
print('CUDA Available:', torch.cuda.is_available())
print('CUDA Version:', torch.version.cuda if torch.cuda.is_available() else 'N/A')
print('GPU Device:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'N/A')
EOF
```

**예상 출력**:
```
PyTorch Version: 2.3.0
CUDA Available: True
CUDA Version: 12.4
GPU Device: Orin
```

## 3. OpenCV 및 Ultralytics 설치

### 3.1 호환 가능한 OpenCV 설치

```bash
# 기존 OpenCV 제거 (있는 경우)
pip3 uninstall opencv-python -y

# NumPy 1.x와 호환되는 OpenCV 설치
pip3 install opencv-python==4.10.0.84
```

### 3.2 torchvision 설치

```bash
# Ultralytics가 요구하는 torchvision 설치
pip3 install --no-deps torchvision==0.18.0
```

### 3.3 Ultralytics (YOLOv8) 설치

```bash
# 먼저 ultralytics만 설치
pip3 install --no-deps ultralytics

# 필요한 의존성 개별 설치
pip3 install PyYAML matplotlib pillow tqdm scipy polars ultralytics-thop
```

### 3.4 전체 설치 확인

```bash
python3 << 'EOF'
from ultralytics import YOLO
import torch
import cv2
import numpy as np

print('='*60)
print('설치 확인')
print('='*60)
print(f'PyTorch: {torch.__version__}')
print(f'OpenCV: {cv2.__version__}')
print(f'NumPy: {np.__version__}')
print(f'CUDA: {torch.cuda.is_available()}')
print(f'GPU: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else "N/A"}')
print('='*60)

# 모델 테스트
model_path = '/home/amap/2025_aa10_ros2_ws/src/AI/ai_lane_detection/runs/segment/train/weights/best.pt'
import os
if os.path.exists(model_path):
    model = YOLO(model_path)
    model.to('cuda:0')  # GPU로 이동
    print('✅ YOLOv8 모델 로드 성공 (GPU)')
    print(f'   Device: {model.device}')
else:
    print('❌ 모델 파일 없음')
EOF
```

## 4. ROS2 패키지 빌드

### 4.1 워크스페이스 빌드

```bash
cd /home/amap/2025_aa10_ros2_ws
colcon build --packages-select ai_lane_detection
```

### 4.2 환경 설정

```bash
source /home/amap/2025_aa10_ros2_ws/install/setup.bash
```

### 4.3 실행

```bash
# 기본 실행
ros2 run ai_lane_detection lane_detection_node

# Launch 파일로 실행
ros2 launch ai_lane_detection lane_detection.launch.py
```

## 5. 성능 최적화 (선택사항)

### 5.1 GPU 클록 최대화

```bash
sudo jetson_clocks
```

### 5.2 TensorRT 변환 (추론 속도 향상)

```python
from ultralytics import YOLO

# 모델을 TensorRT 엔진으로 변환
model = YOLO('runs/segment/train/weights/best.pt')
model.export(format='engine')  # .engine 파일 생성

# 변환된 엔진 사용
model = YOLO('runs/segment/train/weights/best.engine')
```

## 문제 해결

### PyTorch CUDA 미인식

**증상**: `torch.cuda.is_available()` = False

**해결책**:
```bash
# 잘못된 PyTorch 제거
pip3 uninstall torch torchvision -y

# Jetson용 PyTorch 재설치
pip3 install /tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl
```

### NumPy 버전 충돌

**증상**: `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`

**해결책**:
```bash
pip3 install "numpy<2"
pip3 install opencv-python==4.10.0.84
```

### Ultralytics가 PyTorch를 덮어씀

**증상**: `pip install ultralytics` 후 CUDA 작동 안 함

**해결책**:
```bash
# 단계별 설치
pip3 uninstall torch torchvision -y
pip3 install /tmp/torch-2.3.0-cp310-cp310-linux_aarch64.whl
pip3 install --no-deps torchvision==0.18.0
pip3 install --no-deps ultralytics
pip3 install PyYAML matplotlib pillow tqdm scipy polars ultralytics-thop
```

### 메모리 부족

**증상**: 모델 로딩 또는 추론 중 메모리 오류

**해결책**:
```bash
# Swap 메모리 추가
sudo systemctl disable nvzramconfig
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# 영구 설정
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## 검증 스크립트

다음 스크립트로 전체 설치를 검증할 수 있습니다:

```python
#!/usr/bin/env python3

import sys

def check_installation():
    print('='*70)
    print('Jetson Orin Nano - AI Lane Detection 설치 검증')
    print('='*70)

    # PyTorch 확인
    try:
        import torch
        print(f'✅ PyTorch: {torch.__version__}')

        if not torch.cuda.is_available():
            print('❌ CUDA를 사용할 수 없습니다!')
            return False

        print(f'✅ CUDA: {torch.version.cuda}')
        print(f'✅ GPU: {torch.cuda.get_device_name(0)}')
    except Exception as e:
        print(f'❌ PyTorch 오류: {e}')
        return False

    # OpenCV 확인
    try:
        import cv2
        print(f'✅ OpenCV: {cv2.__version__}')
    except Exception as e:
        print(f'❌ OpenCV 오류: {e}')
        return False

    # NumPy 확인
    try:
        import numpy as np
        print(f'✅ NumPy: {np.__version__}')
        if np.__version__.startswith('2'):
            print('⚠️  NumPy 2.x는 호환성 문제가 있을 수 있습니다.')
    except Exception as e:
        print(f'❌ NumPy 오류: {e}')
        return False

    # Ultralytics 확인
    try:
        from ultralytics import YOLO
        print(f'✅ Ultralytics: 설치됨')
    except Exception as e:
        print(f'❌ Ultralytics 오류: {e}')
        return False

    # 모델 로드 테스트
    import os
    model_path = '/home/amap/2025_aa10_ros2_ws/src/AI/ai_lane_detection/runs/segment/train/weights/best.pt'

    if os.path.exists(model_path):
        try:
            model = YOLO(model_path)
            model.to('cuda:0')
            print(f'✅ YOLOv8 모델 로드 및 GPU 이동 성공')
            print(f'   Device: {model.device}')
        except Exception as e:
            print(f'❌ 모델 로드 오류: {e}')
            return False
    else:
        print(f'⚠️  모델 파일 없음: {model_path}')

    print('='*70)
    print('✅ 모든 검증 통과! GPU 기반 YOLOv8 실행 준비 완료')
    print('='*70)
    return True

if __name__ == '__main__':
    success = check_installation()
    sys.exit(0 if success else 1)
```

## 추가 리소스

- [NVIDIA Jetson PyTorch 포럼](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
- [Ultralytics YOLOv8 문서](https://docs.ultralytics.com/)
- [ROS2 Humble 문서](https://docs.ros.org/en/humble/)

## 패키지 정보

- **패키지 위치**: `/home/amap/2025_aa10_ros2_ws/src/AI/ai_lane_detection`
- **모델 파일**: `runs/segment/train/weights/best.pt` (23.08 MB)
- **ROS2 노드**: `lane_detection_node.py` (전통적 CV 방식 - Canny + Hough)
- **YOLOv8 스크립트**: `ai_pilot.py` (DSS SDK용, ROS2 아님)

**참고**: 현재 `lane_detection_node.py`는 YOLOv8을 사용하지 않습니다. YOLOv8 기반 ROS2 노드가 필요한 경우 별도 개발이 필요합니다.
