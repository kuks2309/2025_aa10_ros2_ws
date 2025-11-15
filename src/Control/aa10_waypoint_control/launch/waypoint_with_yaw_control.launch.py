#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    waypoint_control_dir = get_package_share_directory('aa10_waypoint_control')
    yaw_control_dir = get_package_share_directory('aa10_car_yaw_control')

    waypoint_config = os.path.join(waypoint_control_dir, 'config', 'waypoint_control.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Waypoint follower node
    waypoint_follower_node = Node(
        package='aa10_waypoint_control',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[waypoint_config, {'use_sim_time': use_sim_time}],
    )

    # Waypoint publisher node
    waypoint_publisher_node = Node(
        package='aa10_waypoint_control',
        executable='waypoint_publisher.py',
        name='waypoint_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Yaw control node
    yaw_control_node = Node(
        package='aa10_car_yaw_control',
        executable='aa10_car_yaw_control_node',
        name='aa10_car_yaw_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'Kp_waypoint': 0.5,
            'Kd_waypoint': 0.1,
            'Ki_waypoint': 0.0,
            'waypoint_left_angle_max': 42,
            'waypoint_right_angle_max': -42,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        waypoint_follower_node,
        waypoint_publisher_node,
        yaw_control_node,
    ])
