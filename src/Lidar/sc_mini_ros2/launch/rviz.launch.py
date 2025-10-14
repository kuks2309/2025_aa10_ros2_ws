import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'sc_mini_ros2'
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'sc_m.rviz'
    )
    
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])