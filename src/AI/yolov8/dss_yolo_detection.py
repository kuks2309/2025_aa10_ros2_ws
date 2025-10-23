#!/usr/bin/env python3
"""
DSS + YOLOv8 실시간 객체 감지 시스템
DSS 시뮬레이션에서 카메라 스트림을 받아 YOLOv8로 객체 감지
"""

import cv2
import numpy as np
from ultralytics import YOLO
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

class DSSYOLONode(Node):
    def __init__(self):
        super().__init__('dss_yolo_node')
        
        # YOLOv8 모델 초기화 (GPU 사용)
        self.model = YOLO('yolov8n.pt')
        self.model.to('cuda')
        self.get_logger().info('✅ YOLOv8 모델이 GPU에 로드됨')
        
        # OpenCV 브리지
        self.bridge = CvBridge()
        
        # ROS2 구독자/발행자
        self.image_sub = self.create_subscription(
            Image, '/dss/image', self.image_callback, 10)
        
        self.detection_pub = self.create_publisher(
            String, '/dss/detections', 10)
        
        # 성능 측정
        self.frame_count = 0
        self.start_time = time.time()
        
        self.get_logger().info('🚀 DSS-YOLOv8 객체 감지 시스템 시작됨')
    
    def image_callback(self, msg):
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            
            # YOLOv8 객체 감지 (GPU)
            results = self.model(cv_image, device='cuda', verbose=False)
            
            # 감지 결과 처리
            detections = []
            for r in results:
                if len(r.boxes) > 0:
                    for box in r.boxes:
                        conf = float(box.conf[0])
                        cls = int(box.cls[0])
                        name = self.model.names[cls]
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        
                        detection = {
                            'class': name,
                            'confidence': conf,
                            'bbox': [float(x1), float(y1), float(x2), float(y2)]
                        }
                        detections.append(detection)
            
            # 결과 발행
            detection_msg = String()
            detection_msg.data = json.dumps({
                'timestamp': time.time(),
                'detections': detections,
                'count': len(detections)
            })
            self.detection_pub.publish(detection_msg)
            
            # 성능 측정
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # 30프레임마다 FPS 출력
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                self.get_logger().info(
                    f'📊 FPS: {fps:.1f}, 감지된 객체: {len(detections)}')
                
        except Exception as e:
            self.get_logger().error(f'❌ 객체 감지 오류: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DSSYOLONode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\\n⏹️  시스템 종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
