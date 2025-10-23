#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
import time
from ultralytics import YOLO

class YOLOv8LaneDetector:
    def __init__(self, model_path, conf_threshold=0.3):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.model = None
        self.load_model()
    
    def load_model(self):
        """YOLOv8 모델 로드"""
        try:
            if not os.path.exists(self.model_path):
                print(f"❌ 모델 파일을 찾을 수 없습니다: {self.model_path}")
                return False
            
            print(f"📥 YOLOv8 모델 로드 중: {self.model_path}")
            self.model = YOLO(self.model_path)
            print("✅ YOLOv8 모델 로드 완료!")
            return True
        except Exception as e:
            print(f"❌ 모델 로드 실패: {e}")
            return False
    
    def get_class_colors(self):
        """클래스별 색상 정의"""
        colors = {
            0: (255, 0, 0),     # lane - 파란색 (BGR)
            1: (0, 255, 255),   # stop_line - 노란색 (BGR)
        }
        return colors
    
    def get_class_names(self):
        """클래스 이름 정의"""
        class_names = {
            0: "lane",
            1: "stop_line"
        }
        return class_names
    
    def create_segmentation_mask(self, image, results):
        """세그멘테이션 결과에서 lane 마스크만 생성"""
        colors = self.get_class_colors()
        combined_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        
        if results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            confidences = results[0].boxes.conf.cpu().numpy()
            
            for mask, cls, conf in zip(masks, classes, confidences):
                # 신뢰도가 임계값 이상이고 lane(class 0)인 것만 처리
                if conf >= self.conf_threshold and cls == 0:
                    # 마스크를 이미지 크기로 리사이즈
                    mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                    mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255
                    combined_mask = cv2.bitwise_or(combined_mask, mask_binary)
        
        return combined_mask
    
    def create_stop_line_mask(self, image, results):
        """세그멘테이션 결과에서 stop_line 마스크 생성"""
        combined_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        
        if results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            confidences = results[0].boxes.conf.cpu().numpy()
            
            for mask, cls, conf in zip(masks, classes, confidences):
                # 신뢰도가 임계값 이상이고 stop_line(class 1)인 것만 처리
                if conf >= self.conf_threshold and cls == 1:
                    # 마스크를 이미지 크기로 리사이즈
                    mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                    mask_binary = (mask_resized > 0.5).astype(np.uint8) * 255
                    combined_mask = cv2.bitwise_or(combined_mask, mask_binary)
        
        return combined_mask
    
    def detect_stop_line(self, image, results):
        """정지선 감지 및 위치 반환"""
        if results[0].masks is None:
            return -1
        
        masks = results[0].masks.data.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy().astype(int)
        confidences = results[0].boxes.conf.cpu().numpy()
        
        stop_line_positions = []
        
        for mask, cls, conf in zip(masks, classes, confidences):
            # 신뢰도가 임계값 이상이고 stop_line(class 1)인 것만 처리
            if conf >= self.conf_threshold and cls == 1:
                # 마스크를 이미지 크기로 리사이즈
                mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                mask_binary = (mask_resized > 0.5).astype(np.uint8)
                
                # 정지선 픽셀들의 y 좌표 찾기
                y_coords, x_coords = np.where(mask_binary > 0)
                if len(y_coords) > 0:
                    # 정지선의 평균 y 좌표 (가장 가까운 정지선)
                    avg_y = np.mean(y_coords)
                    stop_line_positions.append(avg_y)
        
        if len(stop_line_positions) > 0:
            # 가장 아래쪽(가까운) 정지선 반환
            return max(stop_line_positions)
        else:
            return -1
    
    def create_overlay_image(self, image, lane_mask, stop_line_mask=None, alpha=0.4):
        """마스크를 이미지에 오버레이"""
        overlay_image = image.copy()
        
        # 차선을 파란색으로 오버레이
        if lane_mask is not None and lane_mask.sum() > 0:
            blue_color = (255, 0, 0)  # BGR - 파란색
            overlay = image.copy()
            overlay[lane_mask > 0] = blue_color
            overlay_image = cv2.addWeighted(overlay_image, 1-alpha, overlay, alpha, 0)
        
        # 정지선을 핑크색으로 오버레이
        if stop_line_mask is not None and stop_line_mask.sum() > 0:
            pink_color = (255, 0, 255)  # BGR - 핑크색 (마젠타)
            overlay = overlay_image.copy()
            overlay[stop_line_mask > 0] = pink_color
            overlay_image = cv2.addWeighted(overlay_image, 1-alpha, overlay, alpha, 0)
        
        return overlay_image

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        
        # 파라미터 선언 (기본값은 YAML 파일에서 덮어씌워짐)
        self.declare_parameter('model_path', 'runs/segment/train/weights/best.pt')
        self.declare_parameter('conf_threshold', 0.3)
        self.declare_parameter('anchor_config_file', 'config/anchor_config.json')
        self.declare_parameter('show_debug_image', True)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('default_anchor_lines', [200, 250, 300, 350, 400, 450])
        
        # 차량 영역 파라미터
        self.declare_parameter('car_region.x1_ratio', 0.25)
        self.declare_parameter('car_region.x2_ratio', 0.75)
        self.declare_parameter('car_region.y1_ratio', 0.65)
        self.declare_parameter('car_region.y2_ratio', 0.95)
        
        # 시각화 파라미터
        self.declare_parameter('visualization.overlay_alpha', 0.4)
        self.declare_parameter('visualization.anchor_line_color', [0, 0, 255])
        self.declare_parameter('visualization.center_point_color', [0, 255, 0])
        self.declare_parameter('visualization.center_line_color', [0, 255, 0])
        self.declare_parameter('visualization.anchor_line_thickness', 2)
        self.declare_parameter('visualization.center_line_thickness', 3)
        self.declare_parameter('visualization.center_point_radius', 4)
        
        # 파라미터 가져오기
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        anchor_config_file = self.get_parameter('anchor_config_file').get_parameter_value().string_value
        
        # 상대 경로를 절대 경로로 변환 (asw_predict.py와 동일한 작업 디렉토리 사용)
        pkg_path = '/home/amap/aMAP_ROS2_AA10_Simulator/src/ai_lane_detection'
        if not os.path.isabs(model_path):
            model_path = os.path.join(pkg_path, model_path)
        if not os.path.isabs(anchor_config_file):
            self.anchor_config_file = os.path.join(pkg_path, anchor_config_file)
        else:
            self.anchor_config_file = anchor_config_file
        self.show_debug_image = self.get_parameter('show_debug_image').get_parameter_value().bool_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        
        # 차량 영역 비율 가져오기
        self.car_region_ratios = {
            'x1_ratio': self.get_parameter('car_region.x1_ratio').get_parameter_value().double_value,
            'x2_ratio': self.get_parameter('car_region.x2_ratio').get_parameter_value().double_value,
            'y1_ratio': self.get_parameter('car_region.y1_ratio').get_parameter_value().double_value,
            'y2_ratio': self.get_parameter('car_region.y2_ratio').get_parameter_value().double_value
        }
        
        # 시각화 설정 가져오기
        self.vis_config = {
            'overlay_alpha': self.get_parameter('visualization.overlay_alpha').get_parameter_value().double_value,
            'anchor_line_color': tuple(self.get_parameter('visualization.anchor_line_color').get_parameter_value().integer_array_value),
            'center_point_color': tuple(self.get_parameter('visualization.center_point_color').get_parameter_value().integer_array_value),
            'center_line_color': tuple(self.get_parameter('visualization.center_line_color').get_parameter_value().integer_array_value),
            'anchor_line_thickness': self.get_parameter('visualization.anchor_line_thickness').get_parameter_value().integer_value,
            'center_line_thickness': self.get_parameter('visualization.center_line_thickness').get_parameter_value().integer_value,
            'center_point_radius': self.get_parameter('visualization.center_point_radius').get_parameter_value().integer_value
        }
        
        # YOLOv8 검출기 초기화
        self.lane_detector = YOLOv8LaneDetector(model_path, conf_threshold)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Anchor lines 설정
        self.anchor_lines = []  # 픽셀 위치 (y 좌표)
        self.load_anchor_config()
        
        # 퍼블리셔 및 구독자
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Cross Track Error 퍼블리셔 (차선 중심과 이미지 중심의 차이)
        self.lane_xte_publisher = self.create_publisher(
            Float32,
            '/lane_xte',
            10)
        
        # 정지선 위치 퍼블리셔 (정지선 y좌표, 없으면 -1)
        self.stop_line_publisher = self.create_publisher(
            Float32,
            '/stop_line_position',
            10)
        
        # 디버그 이미지 퍼블리셔 (옵션)
        if self.show_debug_image:
            self.debug_image_publisher = self.create_publisher(
                Image,
                '/lane_detection/debug_image',
                10)
        
        # 성능 측정
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.fps = 0
        
        # OpenCV 윈도우 설정
        if self.show_debug_image:
            # GUI 백엔드를 명시적으로 설정
            os.environ['QT_QPA_PLATFORM'] = 'xcb'
            cv2.namedWindow('Lane Detection Debug', cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow('Lane Detection Debug', 100, 100)
            self.get_logger().info('OpenCV 윈도우 생성됨')
        
        self.get_logger().info('AI Lane Detection Node 시작!')
        self.get_logger().info(f'모델 경로: {model_path}')
        self.get_logger().info(f'Anchor lines: {len(self.anchor_lines)}개')
    
    def load_anchor_config(self):
        """Anchor line 설정 로드"""
        try:
            if os.path.exists(self.anchor_config_file):
                with open(self.anchor_config_file, 'r') as f:
                    config = json.load(f)
                
                if config.get('format') == 'pixels_from_top':
                    self.anchor_lines = config.get('anchor_lines', [])
                else:
                    # 기존 비율 설정을 픽셀 값으로 변환
                    old_ratios = config.get('anchor_lines', [])
                    self.anchor_lines = [int(ratio * self.image_height) for ratio in old_ratios]
                
                self.get_logger().info(f'Anchor 설정 로드: {self.anchor_config_file}')
                self.get_logger().info(f'로드된 anchor lines: {self.anchor_lines}')
            else:
                # 기본 anchor lines 설정
                self.initialize_default_anchor_lines()
                
        except Exception as e:
            self.get_logger().error(f'Anchor 설정 로드 실패: {e}')
            self.initialize_default_anchor_lines()
    
    def initialize_default_anchor_lines(self):
        """기본 anchor lines 초기화"""
        default_positions = self.get_parameter('default_anchor_lines').get_parameter_value().integer_array_value
        self.anchor_lines = list(default_positions)
        self.get_logger().info(f'기본 Anchor lines 초기화: {len(self.anchor_lines)}개')
    
    def get_anchor_y_positions(self, image_height):
        """이미지 높이에 맞는 유효한 anchor y 위치 반환"""
        if self.image_height != image_height:
            self.image_height = image_height
        
        valid_positions = [y for y in self.anchor_lines if 0 <= y < image_height]
        return valid_positions
    
    def detect_car_region(self, image):
        """차량 영역 정의 (YAML 파라미터 사용)"""
        height, width = image.shape[:2]
        
        car_region = {
            'x1': int(width * self.car_region_ratios['x1_ratio']),
            'y1': int(height * self.car_region_ratios['y1_ratio']),
            'x2': int(width * self.car_region_ratios['x2_ratio']),
            'y2': int(height * self.car_region_ratios['y2_ratio'])
        }
        
        return car_region
    
    def draw_anchor_line_excluding_car(self, image, y, width, car_region, color=(0, 0, 255), thickness=2):
        """차량 영역을 제외하고 anchor line 그리기"""
        if y >= car_region['y1'] and y <= car_region['y2']:
            cv2.line(image, (0, y), (car_region['x1'], y), color, thickness)
            cv2.line(image, (car_region['x2'], y), (width, y), color, thickness)
        else:
            cv2.line(image, (0, y), (width, y), color, thickness)
    
    def extract_lane_center_points(self, image, mask):
        """anchor line별로 차선 중심점 추출"""
        if mask is None or mask.sum() == 0:
            return []
        
        height, width = image.shape[:2]
        car_region = self.detect_car_region(image)
        y_positions = self.get_anchor_y_positions(height)
        
        center_points = []
        
        for y in y_positions:
            if y < 0 or y >= height:
                continue
            
            # 해당 y 라인에서 lane 픽셀 찾기
            line_mask = mask[y, :]
            lane_pixels = np.where(line_mask > 0)[0]
            
            if len(lane_pixels) > 0:
                left_most = lane_pixels[0]
                right_most = lane_pixels[-1]
                center_x = (left_most + right_most) // 2
                center_points.append((center_x, y))
        
        return center_points
    
    def draw_debug_visualization(self, image, lane_mask, stop_line_mask, center_points, stop_line_y=-1):
        """디버그 시각화 이미지 생성"""
        # 1. 원본 이미지에 마스크 오버레이 (차선: 파란색, 정지선: 핑크색)
        debug_image = self.lane_detector.create_overlay_image(image, lane_mask, stop_line_mask, alpha=self.vis_config['overlay_alpha'])
        
        height, width = debug_image.shape[:2]
        car_region = self.detect_car_region(debug_image)
        y_positions = self.get_anchor_y_positions(height)
        
        # 2. Anchor lines 그리기
        for i, y in enumerate(y_positions):
            if y < 0 or y >= height:
                continue
            
            line_color = self.vis_config['anchor_line_color']
            self.draw_anchor_line_excluding_car(debug_image, y, width, car_region, line_color, self.vis_config['anchor_line_thickness'])
            
            # Line 번호와 Y좌표 표시
            cv2.putText(debug_image, f"L{i+1}(y={y})", (10, y-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 3. 정지선 위치 수평 라인 그리기
        if stop_line_y > 0:
            stop_line_color = (255, 0, 255)  # 핑크색 (BGR)
            cv2.line(debug_image, (0, int(stop_line_y)), (width, int(stop_line_y)), stop_line_color, 3)
            # 정지선 위치 텍스트 표시
            cv2.putText(debug_image, f"STOP LINE: y={int(stop_line_y)}", (width-200, int(stop_line_y)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, stop_line_color, 2)
        
        # 4. 중심점들 그리기
        for center_x, center_y in center_points:
            cv2.circle(debug_image, (center_x, center_y), self.vis_config['center_point_radius'], self.vis_config['center_point_color'], -1)
        
        # 5. 중심점들을 연결하는 선 그리기
        if len(center_points) >= 2:
            for i in range(len(center_points) - 1):
                cv2.line(debug_image, center_points[i], center_points[i + 1], self.vis_config['center_line_color'], self.vis_config['center_line_thickness'])
        
        # 6. 정보 텍스트 추가
        cv2.putText(debug_image, f'FPS: {self.fps}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(debug_image, f'Center Points: {len(center_points)}', (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_image, f'Anchor Lines: {len(y_positions)}', (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 정지선 상태 표시
        if stop_line_y > 0:
            cv2.putText(debug_image, f'Stop Line: DETECTED', (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        else:
            cv2.putText(debug_image, f'Stop Line: NOT DETECTED', (10, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
        
        return debug_image
    
    def calculate_fps(self):
        """FPS 계산"""
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.fps_start_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.fps_start_time = current_time
    
    def calculate_and_publish_xte(self, image_width, center_points):
        """Cross Track Error 계산 및 퍼블리시"""
        if len(center_points) == 0:
            return
        
        # 이미지 중심
        image_center_x = image_width / 2.0
        
        # 차선 중심점들의 평균 x 좌표 계산 (가중 평균 사용 - 가까운 점에 더 많은 가중치)
        weighted_sum_x = 0.0
        weight_sum = 0.0
        
        for center_x, center_y in center_points:
            # y가 클수록(아래쪽일수록) 더 높은 가중치
            weight = center_y / self.image_height
            weighted_sum_x += center_x * weight
            weight_sum += weight
        
        if weight_sum > 0:
            lane_center_x = weighted_sum_x / weight_sum
            
            # Cross Track Error 계산 (픽셀 단위)
            xte = lane_center_x - image_center_x
            
            # Float32 메시지 생성 및 퍼블리시
            xte_msg = Float32()
            xte_msg.data = float(xte)
            self.lane_xte_publisher.publish(xte_msg)
            
            # 디버그 로깅
            if self.fps_counter % 10 == 0:  # 10프레임마다 한번
                self.get_logger().info(f'XTE: {xte:.1f}px (lane_center: {lane_center_x:.1f}, image_center: {image_center_x:.1f})')
    
    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            start_time = time.time()
            
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # YOLOv8 모델이 로드되지 않은 경우
            if self.lane_detector.model is None:
                self.get_logger().warn_once('YOLOv8 모델이 로드되지 않았습니다. 빈 마스크로 시각화만 표시합니다.')
                # 빈 마스크로 기본 시각화
                lane_mask = np.zeros((cv_image.shape[0], cv_image.shape[1]), dtype=np.uint8)
                stop_line_mask = np.zeros((cv_image.shape[0], cv_image.shape[1]), dtype=np.uint8)
                center_points = []
                stop_line_y = -1
            else:
                # YOLOv8 추론 수행
                results = self.lane_detector.model(cv_image, 
                                                 conf=self.lane_detector.conf_threshold, 
                                                 verbose=False)
                
                # 차선 마스크 생성 (lane 클래스만)
                lane_mask = self.lane_detector.create_segmentation_mask(cv_image, results)
                
                # 정지선 마스크 생성 (stop_line 클래스만)
                stop_line_mask = self.lane_detector.create_stop_line_mask(cv_image, results)
                
                # 차선 중심점 추출
                center_points = self.extract_lane_center_points(cv_image, lane_mask)
                
                # 정지선 위치 계산
                stop_line_y = self.lane_detector.detect_stop_line(cv_image, results)
            
            # 디버그 이미지 생성 및 퍼블리시
            if self.show_debug_image and hasattr(self, 'debug_image_publisher'):
                debug_image = self.draw_debug_visualization(cv_image, lane_mask, stop_line_mask, center_points, stop_line_y)
                
                # OpenCV 윈도우에 표시
                cv2.imshow('Lane Detection Debug', debug_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q 또는 ESC
                    self.get_logger().info('종료 요청')
                    cv2.destroyAllWindows()
                    exit()
                
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                    debug_msg.header = msg.header
                    self.debug_image_publisher.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f'디버그 이미지 변환 실패: {e}')
            
            # Cross Track Error 계산 및 퍼블리시
            self.calculate_and_publish_xte(cv_image.shape[1], center_points)
            
            # 정지선 위치 퍼블리시
            stop_line_msg = Float32()
            stop_line_msg.data = float(stop_line_y)
            self.stop_line_publisher.publish(stop_line_msg)
            
            # 정지선 디버그 로깅
            if self.fps_counter % 10 == 0 and stop_line_y != -1:
                self.get_logger().info(f'정지선 감지: y={stop_line_y:.1f}px')
            
            # FPS 계산
            self.calculate_fps()
            
            # 처리 시간 출력
            end_time = time.time()
            duration_ms = (end_time - start_time) * 1000
            
            if len(center_points) > 0:
                self.get_logger().info(f'차선 중심점 {len(center_points)}개 검출 | '
                                     f'처리시간: {duration_ms:.1f}ms | FPS: {self.fps}')
            
        except Exception as e:
            # cv_bridge 및 일반적인 이미지 변환 오류 처리
            if 'cv_bridge' in str(e) or 'image' in str(e).lower():
                self.get_logger().error(f'이미지 변환 실패: {e}')
            else:
                self.get_logger().error(f'이미지 처리 중 오류: {e}')
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    lane_detection_node = None
    
    try:
        lane_detection_node = LaneDetectionNode()
        rclpy.spin(lane_detection_node)
    except KeyboardInterrupt:
        print('\n키보드 인터럽트로 종료')
    except Exception as e:
        print(f'노드 실행 중 오류: {e}')
    finally:
        cv2.destroyAllWindows()
        if lane_detection_node is not None:
            lane_detection_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()