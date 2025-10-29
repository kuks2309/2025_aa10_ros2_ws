#!/usr/bin/env python3
"""
LiDAR와 TF를 함께 실행하는 통합 Launch 파일
- sc_mini LiDAR 노드 실행
- base_link -> laser_link TF 발행
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. LiDAR 노드
        Node(
            package='sc_mini_ros2',
            executable='sc_mini_ros2',
            name='sc_mini_ros2',
            output='screen',
            parameters=[{
                'frame_id': 'laser_link',
                'port': '/dev/sc_mini',
                'baud_rate': 115200
            }]
        ),

        # 2. TF: base_footprint -> base_link
        # Z축: 7cm (지면에서 로봇 중심까지)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0.07', '0', '0', '0', 'base_footprint', 'base_link']
        ),

        # 3. TF: base_link -> laser_link
        # X축: 9cm (LiDAR가 로봇 앞쪽에 위치)
        # Z축: 0.1cm (약간 위에 위치)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0.09', '0', '0.001', '0', '0', '0', 'base_link', 'laser_link']
        )
    ])
