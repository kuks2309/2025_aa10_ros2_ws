#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    slam_mode = LaunchConfiguration('slam_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_slam_mode = DeclareLaunchArgument(
        'slam_mode',
        default_value='scan_matching',
        choices=['mapping', 'localization', 'scan_matching'],
        description='SLAM mode: mapping for creating maps, localization for using existing maps, scan_matching for Hector-like localization without prior map'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Package directory
    slam_toolbox_config_dir = FindPackageShare('slam_toolbox_config')
    
    # Mapping mode launch
    mapping_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == 'mapping'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        slam_toolbox_config_dir,
                        'launch',
                        'online_async_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        ]
    )
    
    # Localization mode launch
    localization_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == 'localization'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        slam_toolbox_config_dir,
                        'launch',
                        'localization_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        ]
    )
    
    # Scan matching mode launch (Hector-like)
    scan_matching_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == 'scan_matching'"])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        slam_toolbox_config_dir,
                        'launch',
                        'scan_matching_localization.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        ]
    )
    
    return LaunchDescription([
        declare_slam_mode,
        declare_use_sim_time,
        mapping_launch,
        localization_launch,
        scan_matching_launch,
    ])