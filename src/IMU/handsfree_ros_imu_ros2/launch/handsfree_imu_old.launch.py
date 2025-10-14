from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for IMU'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='921600',
        description='Baud rate for IMU serial communication'
    )
    
    gra_normalization_arg = DeclareLaunchArgument(
        'gra_normalization',
        default_value='True',
        description='Normalize gravity acceleration'
    )
    
    return LaunchDescription([
        port_arg,
        baud_arg,
        gra_normalization_arg,
        
        Node(
            package='handsfree_ros_imu_ros2',
            executable='hfi_a9_ros_ros2',
            name='handsfree_imu_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'gra_normalization': LaunchConfiguration('gra_normalization')
            }],
            remappings=[
                ('/IMU_data', '/imu/data'),
                ('/mag_data', '/imu/mag'),
            ]
        )
    ])