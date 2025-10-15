from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('handsfree_ros_imu_ros2')

    return LaunchDescription([
        # Include IMU launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_share,
                    'launch',
                    'handsfree_imu.launch.py'
                ])
            )
        ),
        # Include TF launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_share,
                    'launch',
                    'imu_tf.launch.py'
                ])
            )
        )
    ])
