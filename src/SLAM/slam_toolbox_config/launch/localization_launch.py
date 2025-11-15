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
    map_file_name = LaunchConfiguration('map')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_toolbox_config'),
            'maps',
            'my_map.yaml'
        ]),
        description='Full path to map yaml file to load'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('slam_toolbox_config'),
            'config',
            'mapper_params_localization.yaml'
        ]),
        description='Full path to the SLAM Toolbox parameters file for localization'
    )
    
    # SLAM Toolbox localization node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
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
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file_name
        }]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_slam_params_file_cmd,
        map_server_node,
        lifecycle_manager_node,
        slam_toolbox_node,
    ])