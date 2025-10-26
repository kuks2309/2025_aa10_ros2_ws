# ROI Display Tool

이미지에 사각형 ROI(Region of Interest)를 표시하는 Python 도구입니다.

## 사용법

```bash
# 기본 실행 (기본 경로 사용)
python3 roi_display.py

# 사용자 지정 이미지 폴더와 설정 파일
python3 roi_display.py --image-folder /path/to/images --roi-config my_roi.json
```

## 키보드 조작

- `n` 또는 `Space`: 다음 이미지
- `p`: 이전 이미지  
- `c`: 현재 이미지의 ROI 영역을 crop하여 1/2 크기로 저장
- `a`: 모든 이미지의 ROI 영역을 crop하여 1/2 크기로 저장
- `s`: 현재 이미지를 ROI와 함께 저장
- `q` 또는 `ESC`: 종료

## ROI 설정 (roi_config.json)

### JSON 구조 (단순화)

```json
{
    "x": 150,
    "y": 200,
    "width": 340,
    "height": 200
}
```

### ROI 파라미터

- `x`: ROI 좌상단 X 좌표
- `y`: ROI 좌상단 Y 좌표  
- `width`: ROI 너비
- `height`: ROI 높이

## 필요한 패키지

```bash
pip install opencv-python numpy
```