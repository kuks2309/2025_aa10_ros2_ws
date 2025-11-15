import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('aa10_lidar_cone_maze')

    # Path to the parameter file
    param_file = os.path.join(package_dir, 'config', 'cone_navigation.yaml')

    # Verify file exists and print path for debugging
    print(f"Loading parameters from: {param_file}")
    if os.path.exists(param_file):
        print("Parameter file found!")
    else:
        print("WARNING: Parameter file NOT found!")

    return LaunchDescription([
        Node(
            package='aa10_lidar_cone_maze',
            executable='aa10_cone_navigation',
            name='aa10_cone_navigation',
            namespace='',
            parameters=[param_file],
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])