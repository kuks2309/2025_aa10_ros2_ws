from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aa10_car_yaw_control',
            executable='aa10_car_yaw_control_node',
            name='aa10_car_yaw_control_node',
            output='screen',
            parameters=[{
                'use_imu': True,
                'yaw_control_mode_topic': '/car_control/steering_control_mode',
                'imu_yaw_angle_topic': '/handsfree/imu_yaw_radian',
                'imu_angle_degree_topic': '/handsfree/imu_yaw_correction_degree',
                'yaw_target_topic': '/car_control/target_angle',
                'yaw_control_steering_output_topic': '/car_control/steering_angle',
                'yaw_control_speed_input_topic': '/car_control/speed',
                'vision_cross_track_error_topic': '/xte/vision',
                'maze_xte_topic': '/xte/maze',
                'steer_input_topic': '/xte/steer',
                'Kp_imu_degree': 3.0,
                'Ki_imu_degree': 0.0,
                'Kd_imu_degree': 7.0,
                'Kp_vision': 0.25,
                'Kd_vision': 1.5,
                'Ki_vision': 0.0,
                'Kp_maze': 1.0,
                'Kd_maze': 1.5,
                'Ki_maze': 0.0,
                'vision_xte_right_angle_max': -35,
                'vision_xte_left_angle_max': 35,
                'maze_right_angle_max': -35,
                'maze_left_angle_max': 35
            }]
        )
    ])