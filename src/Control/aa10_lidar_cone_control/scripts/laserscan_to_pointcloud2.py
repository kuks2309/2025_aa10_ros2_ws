#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
import sensor_msgs_py.point_cloud2 as pc2

class LaserScanToPointCloud2(Node):
    def __init__(self):
        super().__init__('laserscan_to_pointcloud2')

        # Create LaserProjection object
        self.laser_projection = LaserProjection()

        # Parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/scan_pointcloud2')
        self.declare_parameter('target_frame', 'laser_link')

        scan_topic = self.get_parameter('scan_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        # Publisher
        self.cloud_pub = self.create_publisher(
            PointCloud2,
            cloud_topic,
            10
        )

        self.get_logger().info(f'LaserScan to PointCloud2 converter started')
        self.get_logger().info(f'Subscribing to: {scan_topic}')
        self.get_logger().info(f'Publishing to: {cloud_topic}')

    def scan_callback(self, scan_msg):
        try:
            # Convert LaserScan to PointCloud2
            cloud_msg = self.laser_projection.projectLaser(scan_msg)

            # Set frame_id
            cloud_msg.header.frame_id = self.target_frame

            # Publish
            self.cloud_pub.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f'Error converting scan to cloud: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
