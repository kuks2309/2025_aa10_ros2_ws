from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='handsfree_ros_imu_ros2',
            executable='hfi_a9_ros_ros2',
            name='hfi_a9_ros_ros2',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 921600,
                'gra_normalization': True
            }]
        )
    ])