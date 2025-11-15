import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('wall_maze')

    # Path to the parameter file
    param_file = os.path.join(package_dir, 'config', 'wall_maze_params.yaml')

    return LaunchDescription([
        Node(
            package='wall_maze',
            executable='wall_maze_node',
            name='wall_maze_node',
            parameters=[param_file],
            output='screen',
            emulate_tty=True
        )
    ])