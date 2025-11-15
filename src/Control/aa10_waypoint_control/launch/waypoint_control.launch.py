#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('aa10_waypoint_control')
    config_file = os.path.join(pkg_dir, 'config', 'waypoint_control.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Waypoint follower node
    waypoint_follower_node = Node(
        package='aa10_waypoint_control',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
    )

    # Waypoint publisher node
    waypoint_publisher_node = Node(
        package='aa10_waypoint_control',
        executable='waypoint_publisher.py',
        name='waypoint_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        waypoint_follower_node,
        waypoint_publisher_node,
    ])
