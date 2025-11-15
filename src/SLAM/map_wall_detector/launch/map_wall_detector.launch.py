#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 파라미터 선언
        DeclareLaunchArgument('min_wall_length', default_value='2.0'),
        DeclareLaunchArgument('max_wall_distance', default_value='5.0'),
        DeclareLaunchArgument('angle_resolution', default_value='1.0'),
        DeclareLaunchArgument('merge_angle_threshold', default_value='5.0'),
        DeclareLaunchArgument('max_peaks_display', default_value='5'),
        DeclareLaunchArgument('wall_threshold', default_value='0.15'),
        DeclareLaunchArgument('analysis_range_x', default_value='5.0'),
        DeclareLaunchArgument('analysis_range_y', default_value='5.0'),
        DeclareLaunchArgument('use_opencv_hough', default_value='true'),
        DeclareLaunchArgument('opencv_rho_resolution', default_value='1.0'),
        DeclareLaunchArgument('opencv_theta_resolution', default_value='1.0'),
        DeclareLaunchArgument('opencv_threshold', default_value='30'),
        DeclareLaunchArgument('image_size', default_value='400'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='map_wall_detector',
            executable='map_wall_detector_node',
            name='map_wall_detector',
            output='screen',
            parameters=[{
                'min_wall_length': LaunchConfiguration('min_wall_length'),
                'max_wall_distance': LaunchConfiguration('max_wall_distance'),
                'angle_resolution': LaunchConfiguration('angle_resolution'),
                'merge_angle_threshold': LaunchConfiguration('merge_angle_threshold'),
                'max_peaks_display': LaunchConfiguration('max_peaks_display'),
                'distance_resolution': 0.05,
                'wall_threshold': LaunchConfiguration('wall_threshold'),
                'analysis_range_x': LaunchConfiguration('analysis_range_x'),
                'analysis_range_y': LaunchConfiguration('analysis_range_y'),
                'use_opencv_hough': LaunchConfiguration('use_opencv_hough'),
                'opencv_rho_resolution': LaunchConfiguration('opencv_rho_resolution'),
                'opencv_theta_resolution': LaunchConfiguration('opencv_theta_resolution'),
                'opencv_threshold': LaunchConfiguration('opencv_threshold'),
                'image_size': LaunchConfiguration('image_size'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])