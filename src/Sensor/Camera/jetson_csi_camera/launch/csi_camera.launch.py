from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='CSI camera sensor ID (0 or 1)'
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='1280',
            description='Image width'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='720',
            description='Image height'
        ),
        DeclareLaunchArgument(
            'framerate',
            default_value='30',
            description='Camera framerate (fps)'
        ),
        DeclareLaunchArgument(
            'flip_method',
            default_value='0',
            description='Flip method (0=none, 2=rotate-180)'
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='camera_link',
            description='Camera frame ID'
        ),
        DeclareLaunchArgument(
            'image_scale',
            default_value='1.0',
            description='Image scaling factor (0.5 = 50%, 1.0 = 100%)'
        ),

        Node(
            package='jetson_csi_camera',
            executable='csi_camera_node',
            name='csi_camera_node',
            output='screen',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'framerate': LaunchConfiguration('framerate'),
                'flip_method': LaunchConfiguration('flip_method'),
                'camera_frame_id': LaunchConfiguration('camera_frame_id'),
                'image_scale': LaunchConfiguration('image_scale'),
                'publish_rate': 30.0,
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ]
        )
    ])
