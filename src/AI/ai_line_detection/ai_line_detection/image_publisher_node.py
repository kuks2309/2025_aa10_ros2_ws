#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Image Publisher Node for Testing AI Line Detection
Publishes images from test_image/images folder to /camera/image_raw topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob


class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')

        # Parameters
        self.declare_parameter('image_folder', 'test_image/images')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('loop', True)
        self.declare_parameter('camera_topic', '/camera/ai_lane_detect')

        image_folder = self.get_parameter('image_folder').value
        publish_rate = self.get_parameter('publish_rate').value
        self.loop = self.get_parameter('loop').value
        camera_topic = self.get_parameter('camera_topic').value

        # Get absolute path for image folder
        if not os.path.isabs(image_folder):
            # Try source directory first (test_image is not installed, only in source)
            src_dir = '/home/amap/2025_aa10_ros2_ws/src/AI/ai_line_detection'
            test_folder = os.path.join(src_dir, image_folder)

            if os.path.exists(test_folder):
                image_folder = test_folder
                self.get_logger().info(f'Using source directory: {image_folder}')
            else:
                # Fallback: try relative to current file
                package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                image_folder = os.path.join(package_dir, image_folder)
                self.get_logger().warn(f'Source directory not found, trying: {image_folder}')

        self.get_logger().info(f'Image folder: {image_folder}')
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
        self.get_logger().info(f'Loop: {self.loop}')
        self.get_logger().info(f'Camera topic: {camera_topic}')

        # Load image files
        self.image_files = sorted(glob.glob(os.path.join(image_folder, '*.jpg')))
        if len(self.image_files) == 0:
            self.get_logger().error(f'No images found in {image_folder}')
            return

        self.get_logger().info(f'Found {len(self.image_files)} images')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Current image index
        self.current_idx = 0

        # Publisher
        self.image_pub = self.create_publisher(
            Image,
            camera_topic,
            10
        )

        # Timer to publish images
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Image Publisher Node initialized')

    def timer_callback(self):
        """Publish next image"""
        if len(self.image_files) == 0:
            return

        # Load image
        image_path = self.image_files[self.current_idx]
        cv_image = cv2.imread(image_path)

        if cv_image is None:
            self.get_logger().warn(f'Failed to load image: {image_path}')
            self.current_idx = (self.current_idx + 1) % len(self.image_files)
            return

        # Convert to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera'

        # Publish
        self.image_pub.publish(image_msg)

        # Log progress
        filename = os.path.basename(image_path)
        self.get_logger().info(
            f'[{self.current_idx + 1}/{len(self.image_files)}] Published: {filename}',
            throttle_duration_sec=1.0
        )

        # Move to next image
        self.current_idx += 1

        # Check if reached end
        if self.current_idx >= len(self.image_files):
            if self.loop:
                self.get_logger().info('Reached end, looping back to start')
                self.current_idx = 0
            else:
                self.get_logger().info('Reached end of image sequence, stopping')
                self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Exception during spin: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
