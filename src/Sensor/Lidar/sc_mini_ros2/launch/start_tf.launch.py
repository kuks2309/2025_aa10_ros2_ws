from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0.09', '0', '0', '0', '0', '0', 'base_link', 'laser_link']
        ),
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