#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    """
    Launch file for LiDAR cone detection and tracking control
    Supports both PointCloud2 and LaserScan inputs

    Usage:
    - For PointCloud2: ros2 launch aa10_lidar_cone_control lidar_cone_control.launch.py use_pointcloud2:=true lidar2d_cloud_topic:=/pointcloud2d
    - For LaserScan:   ros2 launch aa10_lidar_cone_control lidar_cone_control.launch.py use_pointcloud2:=false
    """

    # Declare launch arguments
    use_pointcloud2_arg = DeclareLaunchArgument(
        'use_pointcloud2',
        default_value='false',
        description='Use PointCloud2 input (true) or LaserScan input (false)'
    )

    lidar2d_cloud_topic_arg = DeclareLaunchArgument(
        'lidar2d_cloud_topic',
        default_value='/scan_pointcloud2',
        description='Topic name for 2D LiDAR point cloud (if use_pointcloud2=true, use /pointcloud2d)'
    )

    imu_heading_angle_topic_arg = DeclareLaunchArgument(
        'imu_heading_angle_topic',
        default_value='/handsfree/imu_yaw_correction_degree',
        description='Topic name for IMU heading angle'
    )

    min_blob_area_arg = DeclareLaunchArgument(
        'min_blob_area',
        default_value='250',
        description='Minimum blob area for cone detection (pixels)'
    )

    max_blob_area_arg = DeclareLaunchArgument(
        'max_blob_area',
        default_value='600',
        description='Maximum blob area for cone detection (pixels)'
    )

    image_scale_arg = DeclareLaunchArgument(
        'image_scale',
        default_value='400',
        description='Image scale factor (pixels per meter)'
    )

    cone_spacing_min_arg = DeclareLaunchArgument(
        'cone_spacing_min',
        default_value='0.15',
        description='Minimum expected spacing between cones (meters)'
    )

    cone_spacing_max_arg = DeclareLaunchArgument(
        'cone_spacing_max',
        default_value='0.35',
        description='Maximum expected spacing between cones (meters)'
    )

    # LaserScan to PointCloud2 converter (only launched when use_pointcloud2=false)
    # This converts /scan (LaserScan) to /scan_pointcloud2 (PointCloud2)
    laserscan_to_pointcloud_node = Node(
        package='aa10_lidar_cone_control',
        executable='laserscan_to_pointcloud2.py',
        name='laserscan_to_pointcloud2',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_pointcloud2')),
        parameters=[{
            'scan_topic': '/scan',
            'cloud_topic': '/scan_pointcloud2',
            'target_frame': 'laser_link',
        }]
    )

    # Create lidar cone control node
    lidar_cone_control_node = Node(
        package='aa10_lidar_cone_control',
        executable='lidar_cone_control_node',
        name='lidar_cone_control',
        output='screen',
        parameters=[{
            'lidar2d_cloud_topic': LaunchConfiguration('lidar2d_cloud_topic'),
            'imu_heading_angle_topic': LaunchConfiguration('imu_heading_angle_topic'),
            'min_blob_area': LaunchConfiguration('min_blob_area'),
            'max_blob_area': LaunchConfiguration('max_blob_area'),
            'image_scale': LaunchConfiguration('image_scale'),
            'cone_spacing_min': LaunchConfiguration('cone_spacing_min'),
            'cone_spacing_max': LaunchConfiguration('cone_spacing_max'),
        }]
    )

    return LaunchDescription([
        use_pointcloud2_arg,
        lidar2d_cloud_topic_arg,
        imu_heading_angle_topic_arg,
        min_blob_area_arg,
        max_blob_area_arg,
        image_scale_arg,
        cone_spacing_min_arg,
        cone_spacing_max_arg,
        laserscan_to_pointcloud_node,
        lidar_cone_control_node
    ])
