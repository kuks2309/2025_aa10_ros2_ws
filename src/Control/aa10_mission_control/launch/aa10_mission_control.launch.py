import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the config file
    config_file = os.path.join(
        get_package_share_directory('aa10_mission_control'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='aa10_mission_control',
            executable='aa10_mission_control_node',
            name='aa10_mission_control_node',
            output='screen',
            parameters=[config_file],
            emulate_tty=True
        )
    ])
