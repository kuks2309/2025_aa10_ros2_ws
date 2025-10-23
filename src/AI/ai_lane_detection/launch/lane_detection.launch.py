#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from pathlib import Path

def generate_launch_description():
    # 패키지 경로
    pkg_path = Path(__file__).parent.parent
    
    # YAML 파일 경로
    config_file = pkg_path / 'config' / 'lane_detection_params.yaml'
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=str(config_file),
        description='Path to the configuration YAML file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # AI Lane Detection Node
    lane_detection_node = Node(
        package='ai_lane_detection',
        executable='lane_detection_node',
        name='lane_detection_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/lane_detection/debug_image', '/lane_detection/debug_image'),
            ('/lane_xte', '/lane_xte'),
            ('/stop_line_position', '/stop_line_position')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        lane_detection_node
    ])