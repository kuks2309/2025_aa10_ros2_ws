#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Launch file to test AI line detection with test images
Launches both image_publisher_node and ai_line_detection_node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ai_line_detection')

    # Launch arguments
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='5.0',
        description='Image publishing rate (Hz)'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='True',
        description='Loop through images'
    )

    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.3',
        description='YOLO confidence threshold'
    )

    overlay_alpha_arg = DeclareLaunchArgument(
        'overlay_alpha',
        default_value='0.5',
        description='Overlay transparency (0.0-1.0)'
    )

    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='True',
        description='Show OpenCV window for visualization'
    )

    # Image Publisher Node
    image_publisher_node = Node(
        package='ai_line_detection',
        executable='image_publisher_node',
        name='image_publisher',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'loop': LaunchConfiguration('loop'),
            'camera_topic': '/camera/ai_lane_detect'
        }]
    )

    # AI Line Detection Node
    ai_line_detection_node = Node(
        package='ai_line_detection',
        executable='ai_line_detection_node',
        name='ai_line_detection',
        output='screen',
        parameters=[{
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'overlay_alpha': LaunchConfiguration('overlay_alpha'),
            'show_window': LaunchConfiguration('show_window'),
            'camera_topic': '/camera/ai_lane_detect'
        }]
    )

    return LaunchDescription([
        publish_rate_arg,
        loop_arg,
        conf_threshold_arg,
        overlay_alpha_arg,
        show_window_arg,
        image_publisher_node,
        ai_line_detection_node
    ])
