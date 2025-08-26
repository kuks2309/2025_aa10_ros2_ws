from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sc_mini_ros2',
            executable='sc_mini_ros2',
            name='sc_mini_ros2',
            output='screen',
            parameters=[{
                'frame_id': 'laser_link',
                'port': '/dev/sc_mini',
                'baud_rate': 115200
            }]
        )
    ])