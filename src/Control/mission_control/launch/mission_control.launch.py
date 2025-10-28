import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('mission_control')
    
    # Path to the params file
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    
    # Create the node with params file
    mission_control_node = Node(
        package='mission_control',
        executable='mission_control_node',
        name='mission_control_node',
        output='screen',
        parameters=[params_file]
    )
    
    return LaunchDescription([
        mission_control_node
    ])