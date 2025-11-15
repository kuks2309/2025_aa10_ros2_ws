#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    slam_toolbox_config_dir = get_package_share_directory('slam_toolbox_config')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_toolbox_config'),
            'config',
            'scan_matching_localization.yaml'
        ]),
        description='Full path to scan matching localization parameters file'
    )
    
    # SLAM Toolbox node in scan-matching mode (like Hector SLAM)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('/map', '/map'),
        ]
    )
    
    # No need for additional static transforms - Gazebo provides all TF
    # TF tree: map -> odom -> base_footprint -> base_link -> chassis_link -> laser_link
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_slam_params_file_cmd,
        slam_toolbox_node,
    ])