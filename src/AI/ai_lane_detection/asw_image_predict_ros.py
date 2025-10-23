#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os
import time

class ASWImagePredictor(Node):
    def __init__(self):
        super().__init__('asw_image_predictor')
        
        # 파라미터 설정 (asw_predict.py와 동일)
        model_path = 'runs/segment/train/weights/best.pt'
        
        # 상대 경로 처리 (asw_predict.py와 동일)
        if not os.path.isabs(model_path):
            pkg_path = '/home/amap/aMAP_ROS2_AA10_Simulator/src/ai_lane_detection'
            model_path = os.path.join(pkg_path, model_path)
        
        # YOLOv8 모델 로드
        self.model = self.load_trained_model(model_path)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # 구독자
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 퍼블리셔 (결과 이미지)
        self.result_image_publisher = self.create_publisher(
            Image,
            '/asw_prediction/result_image',
            10)
        
        # 성능 측정
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.fps = 0
        self.frame_count = 0
        
        # 종료 플래그
        self.shutdown_requested = False
        
        # OpenCV 윈도우 설정
        cv2.namedWindow('ASW Lane Detection Results', cv2.WINDOW_AUTOSIZE)
        
        self.get_logger().info('ASW Image Predictor Node 시작!')
        self.get_logger().info(f'모델 경로: {model_path}')
        
    def load_trained_model(self, model_path):
        """학습된 모델 로드 (asw_predict.py와 동일)"""
        if not os.path.exists(model_path):
            self.get_logger().error(f"모델 파일을 찾을 수 없습니다: {model_path}")
            return None
        
        try:
            self.get_logger().info(f"YOLOv8 모델 로드 중: {model_path}")
            model = YOLO(model_path)
            self.get_logger().info("✅ YOLOv8 모델 로드 완료!")
            return model
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            return None
    
    def get_class_colors(self):
        """각 클래스별 색상 정의 (asw_predict.py와 동일)"""
        colors = {
            0: (255, 0, 0),     # lane - 파란색 (BGR)
            1: (0, 255, 255),   # stop_line - 노란색 (BGR)
        }
        return colors
    
    def get_class_names(self):
        """클래스 이름 정의 (asw_predict.py와 동일)"""
        class_names = {
            0: "lane",
            1: "stop_line"
        }
        return class_names
    
    def draw_segmentation_results(self, image, results):
        """세그멘테이션 결과를 이미지에 그리기 (asw_predict.py와 완전히 동일)"""
        colors = self.get_class_colors()
        class_names = self.get_class_names()
        
        # 결과 이미지 복사
        result_img = image.copy()
        
        if results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            confidences = results[0].boxes.conf.cpu().numpy()
            
            self.get_logger().info(f"✅ 감지된 객체 수: {len(masks)}")
            
            for mask, cls, conf in zip(masks, classes, confidences):
                # 신뢰도가 0.3 이상인 것만 표시
                if conf < 0.3:
                    continue
                    
                # 매칭되는 클래스만 처리 (lane=0, stop_line=1)
                if cls not in colors:
                    self.get_logger().info(f"알 수 없는 클래스 ID {cls} 건너뜀")
                    continue
                    
                color = colors[cls]
                class_name = class_names[cls]
                
                self.get_logger().info(f"   - {class_name}: 신뢰도 {conf:.2f}")
                
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
            self.get_logger().info("❌ 감지된 객체가 없습니다.")
        
        return result_img
    
    def add_basic_info_overlay(self, image):
        """이미지에 기본 정보 오버레이 추가 (asw_predict.py 스타일 유지)"""
        info_image = image.copy()
        
        # FPS 표시 (작게)
        cv2.putText(info_image, f'FPS: {self.fps}', (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # 프레임 카운트 표시 (작게)
        cv2.putText(info_image, f'Frame: {self.frame_count}', (10, 45), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 조작법 표시 (작게, 하단)
        cv2.putText(info_image, 'Q: quit', 
                   (10, image.shape[0] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        return info_image
    
    def calculate_fps(self):
        """FPS 계산"""
        self.fps_counter += 1
        current_time = time.time()
        if current_time - self.fps_start_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.fps_start_time = current_time
    
    
    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            start_time = time.time()
            
            # 이미지 정보 로깅 (디버깅용)
            if self.frame_count % 50 == 0:  # 50프레임마다 한번
                self.get_logger().info(f"수신된 이미지: {msg.width}x{msg.height}, 인코딩: {msg.encoding}")
            
            # ROS 이미지를 OpenCV 이미지로 변환 (rgb8 -> bgr8 올바른 변환)
            if msg.encoding == "rgb8":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame_count += 1
            
            # 첫 번째 프레임 저장하여 확인 (디버깅용)
            if self.frame_count == 1:
                debug_path = "/home/amap/aMAP_ROS2_AA10_Simulator/src/ai_lane_detection/debug_frame_1.jpg"
                cv2.imwrite(debug_path, cv_image)
                self.get_logger().info(f"첫 번째 프레임 저장: {debug_path}")
            
            # 모델이 로드되지 않은 경우
            if self.model is None:
                self.get_logger().warn_once('YOLOv8 모델이 로드되지 않았습니다.')
                cv2.imshow('ASW Lane Detection Results', cv_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:  # Q 또는 ESC
                    self.get_logger().info('종료 요청')
                    self.shutdown_requested = True
                return
            
            # 640x480으로 고정 리사이즈 (asw_predict.py와 동일)
            FIXED_WIDTH = 640
            FIXED_HEIGHT = 480
            processed_image = cv2.resize(cv_image, (FIXED_WIDTH, FIXED_HEIGHT))
            
            # 전처리된 이미지 저장 (매 100프레임마다, 디버깅용)
            if self.frame_count % 100 == 0:
                debug_processed_path = f"/home/amap/aMAP_ROS2_AA10_Simulator/src/ai_lane_detection/debug_processed_{self.frame_count}.jpg"
                cv2.imwrite(debug_processed_path, processed_image)
                self.get_logger().info(f"전처리된 이미지 저장: {debug_processed_path}")
            
            # YOLOv8 추론 수행 (asw_predict.py와 완전히 동일한 파라미터)
            results = self.model(processed_image, 
                               conf=0.3, 
                               iou=0.5, 
                               imgsz=(FIXED_WIDTH, FIXED_HEIGHT),
                               verbose=False)
            
            # 결과 그리기 (asw_predict.py와 동일)
            result_img = self.draw_segmentation_results(processed_image, results)
            
            # 기본 정보만 추가 (FPS, 크기 등)
            final_image = self.add_basic_info_overlay(result_img)
            
            # FPS 계산
            self.calculate_fps()
            
            # 결과 이미지 퍼블리시
            try:
                result_msg = self.bridge.cv2_to_imgmsg(final_image, "bgr8")
                result_msg.header = msg.header
                self.result_image_publisher.publish(result_msg)
            except Exception as e:
                self.get_logger().error(f'결과 이미지 퍼블리시 실패: {e}')
            
            # 윈도우에 표시
            cv2.imshow('ASW Lane Detection Results', final_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == 27:  # Q 또는 ESC
                self.get_logger().info('종료 요청')
                cv2.destroyAllWindows()
                self.shutdown_requested = True
                return
            elif key == ord('s'):  # S - 현재 프레임 저장 (디버깅용)
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                save_path = f"/home/amap/aMAP_ROS2_AA10_Simulator/src/ai_lane_detection/debug_save_{timestamp}.jpg"
                cv2.imwrite(save_path, processed_image)
                self.get_logger().info(f"현재 프레임 저장: {save_path}")
            
            # 처리 시간 출력 (간단하게)
            end_time = time.time()
            duration_ms = (end_time - start_time) * 1000
            
            # 매 10프레임마다 한 번만 로그 출력 (로그 스팸 방지)
            if self.frame_count % 10 == 0:
                self.get_logger().info(f'Frame {self.frame_count} | 처리시간: {duration_ms:.1f}ms | FPS: {self.fps}')
            
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류: {e}')
    
    def __del__(self):
        try:
            cv2.destroyAllWindows()
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        predictor_node = ASWImagePredictor()
        
        print("🚗 ASW 차선 및 정지선 실시간 검출 시스템")
        print("📌 이미지 크기: 640x480으로 고정")
        print("🎯 검출 대상: Lane(파란색), Stop Line(노란색)")
        print("🎮 조작법: Q/ESC(종료)")
        print("📡 퍼블리시: /asw_prediction/result_image")
        
        # 종료 플래그를 확인하면서 spin
        while rclpy.ok() and not predictor_node.shutdown_requested:
            rclpy.spin_once(predictor_node, timeout_sec=0.1)
        
    except KeyboardInterrupt:
        print('\n키보드 인터럽트로 종료')
    except Exception as e:
        print(f'노드 실행 중 오류: {e}')
    finally:
        # 안전한 종료 처리
        try:
            if 'predictor_node' in locals():
                predictor_node.destroy_node()
        except:
            pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()