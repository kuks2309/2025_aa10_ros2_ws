#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('front_obstacle_detect')

    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'front_obstacle_detect_roi.json')

    # Front obstacle detect node
    obstacle_detect_node = Node(
        package='front_obstacle_detect',
        executable='front_obstacle_detect_node',
        name='front_obstacle_detect_node',
        output='screen',
        parameters=[{
            'config_file': config_file
        }],
        remappings=[
            # Add any topic remappings here if needed
            # ('/scan', '/your_lidar_topic'),
        ]
    )

    return LaunchDescription([
        obstacle_detect_node
    ])
