from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ai_line_detection')

    # Default model path
    default_model_path = os.path.join(pkg_dir, 'weights', 'best.pt')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='Path to YOLOv8 segmentation model'
        ),
        DeclareLaunchArgument(
            'conf_threshold',
            default_value='0.3',
            description='Confidence threshold for detection'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/ai_lane_detect',
            description='Camera image topic to subscribe'
        ),
        DeclareLaunchArgument(
            'overlay_alpha',
            default_value='0.5',
            description='Overlay transparency (0.0-1.0)'
        ),

        Node(
            package='ai_line_detection',
            executable='ai_line_detection_node',
            name='ai_line_detection_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'overlay_alpha': LaunchConfiguration('overlay_alpha'),
            }],
            emulate_tty=True
        )
    ])
