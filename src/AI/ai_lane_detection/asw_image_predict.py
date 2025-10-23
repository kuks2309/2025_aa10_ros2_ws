import cv2
import numpy as np
from ultralytics import YOLO
import os
import glob
from pathlib import Path

def load_trained_model():
    """학습된 모델 로드 (고정 경로 사용)"""
    # 고정된 경로 사용
    model_dir = "runs/segment/train/weights"
    
    if not os.path.exists(model_dir):
        print(f"❌ 모델 폴더를 찾을 수 없습니다: {model_dir}")
        print("먼저 학습을 완료해주세요.")
        return None
    
    # best.pt 우선, 없으면 last.pt 사용
    best_model_path = os.path.join(model_dir, "best.pt")
    last_model_path = os.path.join(model_dir, "last.pt")
    
    if os.path.exists(best_model_path):
        model_path = best_model_path
        print(f"✅ best.pt 모델 로드: {model_path}")
    elif os.path.exists(last_model_path):
        model_path = last_model_path
        print(f"✅ last.pt 모델 로드: {model_path}")
    else:
        print(f"❌ 모델 파일을 찾을 수 없습니다: {model_dir}")
        print("best.pt 또는 last.pt 파일이 필요합니다.")
        return None
    
    try:
        model = YOLO(model_path)
        print(f"✅ 모델 로드 성공! (GPU 모드)")
        return model
    except Exception as e:
        print(f"❌ 모델 로드 실패: {e}")
        return None

def get_class_colors():
    """각 클래스별 색상 정의"""
    colors = {
        0: (255, 0, 0),     # lane - 파란색 (BGR)
        1: (0, 255, 255),   # stop_line - 노란색 (BGR)
    }
    return colors

def get_class_names():
    """클래스 이름 정의"""
    class_names = {
        0: "lane",
        1: "stop_line"
    }
    return class_names

def draw_segmentation_results(image, results):
    """세그멘테이션 결과를 이미지에 그리기"""
    colors = get_class_colors()
    class_names = get_class_names()
    
    # 결과 이미지 복사
    result_img = image.copy()
    
    if results[0].masks is not None:
        masks = results[0].masks.data.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy().astype(int)
        confidences = results[0].boxes.conf.cpu().numpy()
        
        print(f"✅ 감지된 객체 수: {len(masks)}")
        
        for mask, cls, conf in zip(masks, classes, confidences):
            # 신뢰도가 0.3 이상인 것만 표시
            if conf < 0.3:
                continue
                
            color = colors.get(cls, (128, 128, 128))
            class_name = class_names.get(cls, f"class_{cls}")
            
            print(f"   - {class_name}: 신뢰도 {conf:.2f}")
            
            # 마스크를 이미지 크기로 리사이즈
            mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
            
            # 마스크 영역에 색상 오버레이
            overlay = result_img.copy()
            overlay[mask_resized > 0.5] = color
            
            # 투명도 조절 (lane과 stop_line에 대해 다르게 설정)
            if cls == 0:  # lane
                alpha = 0.4  # lane은 좀 더 투명하게
            else:  # stop_line
                alpha = 0.5  # stop_line은 좀 더 진하게
            
            result_img = cv2.addWeighted(result_img, 1-alpha, overlay, alpha, 0)
    
    else:
        print("❌ 감지된 객체가 없습니다.")
    
    return result_img

def process_test_images(model, test_dir="/home/amap/yolov8_asw/dataset/test/images"):
    """테스트 이미지들을 처리 (640x480 고정 크기)"""
    
    # 고정 이미지 크기 (640x480)
    FIXED_WIDTH = 640
    FIXED_HEIGHT = 480
    
    # 테스트 이미지 파일들 찾기
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
    image_files = []
    
    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(test_dir, ext)))
    
    if not image_files:
        print(f"❌ {test_dir}에서 이미지 파일을 찾을 수 없습니다.")
        return
    
    print(f"✅ {len(image_files)}개의 테스트 이미지를 찾았습니다.")
    
    for i, image_path in enumerate(image_files):
        print(f"\n🔍 처리 중: {os.path.basename(image_path)} ({i+1}/{len(image_files)})")
        
        # 이미지 로드
        original_image = cv2.imread(image_path)
        if original_image is None:
            print(f"❌ 이미지를 읽을 수 없습니다: {image_path}")
            continue
        
        print(f"   원본 이미지 크기: {original_image.shape[1]}x{original_image.shape[0]}")
        
        # 640x480으로 고정 리사이즈 (학습 모델과 동일한 크기)
        image = cv2.resize(original_image, (FIXED_WIDTH, FIXED_HEIGHT))
        print(f"   리사이즈된 크기: {FIXED_WIDTH}x{FIXED_HEIGHT}")
        
        # 예측 수행 (640x480 고정 크기로 예측)
        results = model(image, conf=0.3, iou=0.5, imgsz=(FIXED_WIDTH, FIXED_HEIGHT))
        
        # 결과 그리기
        result_img = draw_segmentation_results(image, results)
        
        # 이미지 화면에 표시 (선택사항)
        # 이미지가 너무 크면 리사이즈
        display_img = result_img.copy()
        height, width = display_img.shape[:2]
        
        if width > 1200 or height > 800:
            scale = min(1200/width, 800/height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            display_img = cv2.resize(display_img, (new_width, new_height))
        
        # 원본과 결과 비교 표시
        original_resized = cv2.resize(image, (display_img.shape[1], display_img.shape[0]))
        comparison = np.hstack([original_resized, display_img])
        
        cv2.imshow(f'Original vs Result - {os.path.basename(image_path)}', comparison)
        
        print("   ⌨️  'q': 종료, 'n': 다음 이미지")
        key = cv2.waitKey(0) & 0xFF
        
        cv2.destroyAllWindows()
        
        if key == ord('q'):
            print("🔚 사용자가 종료를 선택했습니다.")
            break

def main():
    """메인 함수"""
    print("🚗 차선 및 정지선 세그멘테이션 (640x480 고정 크기)")
    print("=" * 70)
    print("📌 이미지 크기: 640x480으로 고정 (학습 모델과 동일)")
    print("🎯 검출 대상:")
    print("   - Lane (차선): 파란색 오버레이")
    print("   - Stop Line (정지선): 노란색 오버레이")
    
    # 모델 로드
    model = load_trained_model()
    if model is None:
        return
    
    # 클래스 정보 출력
    print("\n📋 클래스 정보:")
    class_names = get_class_names()
    colors = get_class_colors()
    
    for cls_id, name in class_names.items():
        color_bgr = colors[cls_id]
        color_name = "파란색" if cls_id == 0 else "노란색"
        print(f"   {cls_id}: {name} - {color_name} BGR{color_bgr}")
    
    print("\n🎯 테스트 이미지 처리 시작...")
    
    # 테스트 이미지 처리
    process_test_images(model)
    
    print("\n✅ 모든 처리가 완료되었습니다!")

if __name__ == "__main__":
    main()