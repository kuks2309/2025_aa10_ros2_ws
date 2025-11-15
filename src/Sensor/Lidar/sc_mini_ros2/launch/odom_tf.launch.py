#!/usr/bin/env python3
"""
Odometry TF 발행 (정지 상태용)
실제 로봇이 움직일 때는 odometry 노드가 이 TF를 제공해야 함
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # odom -> base_link (정지 상태)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        )
    ])
