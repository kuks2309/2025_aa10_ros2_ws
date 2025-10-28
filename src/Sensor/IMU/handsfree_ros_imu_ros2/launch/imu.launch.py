#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/HFRobotIMU',
            description='Serial port for HandsFree IMU'
        ),

        DeclareLaunchArgument(
            'baud',
            default_value='921600',
            description='Baud rate for serial communication'
        ),

        Node(
            package='handsfree_ros_imu_ros2',
            executable='hfi_a9_ros_ros2',
            name='handsfree_imu_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'gra_normalization': True,
                'frame_id': 'imu_link'
            }]
        )
    ])
