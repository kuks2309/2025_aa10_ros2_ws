#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "amap_powerpack_single_driver/msg/steering_angle.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <math.h>
#include <iostream>

#define _USE_MATH_DEFINES

#define RAD2DEG(x) ((x) * 180. / M_PI)
#define DEG2RAD(x) ((x) / 180. * M_PI)

#define MAX_angluar_velocity M_PI / 10

#define IMU_CONTROL 0
#define LANE_CONTROL 1
#define MAZE_CONTROL 2
#define STEER_CONTROL 3
#define LIDAR_CONTROL 4

using namespace std;

double roll, pitch, yaw;
double roll_d, pitch_d, yaw_d, yaw_d_360;
double yaw_d_old = 0.;
double target_yaw = 0.;

double Kp_imu_degree = 0.01;
double Kd_imu_degree = 0.04;
double Ki_imu_degree = 0.00;

double Kp_vision = 0.01;
double Kd_vision = 0.04;
double Ki_vision = 0.00;

double Kp_maze = 0.01;
double Kd_maze = 0.04;
double Ki_maze = 0.00;

double Kp_lidar = 0.01;
double Kd_lidar = 0.04;
double Ki_lidar = 0.00;

double error_imu_degree = 0.0;
double error_imu_degree_old = 0.0;

double error_vision = 0.0;
double error_vision_old = 0.0;
double error_vision_sum = 0.0;

double error_maze = 0.0;
double error_maze_old = 0.0;
double error_maze_sum = 0.0;

double error_lidar = 0.0;
double error_lidar_old = 0.0;
double error_lidar_sum = 0.0;

double imu_heading_anlge_degree = 0.0;
double slam_pose_yaw_degree = 0.0;
double target_slam_yaw_degree = 0.0;

bool control_action_flag = 0;
bool use_imu = false;

int vision_xte_right_angle_max = -42;
int vision_xte_left_angle_max = 42;

int maze_right_angle_max = -42;
int maze_left_angle_max = 42;

int steering_control_mode = -1;
int car_speed = 0;
int steer_input = 0;
double target_yaw_degree = 0.0;

double vision_cross_track_error = 0.0;
double vision_xte_offset = 0.0;  // Vision XTE offset for adjustment
double maze_xte = 0.0;

bool yaw_control_complete_flag = false;

void yaw_control_speed_input_Callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    car_speed = msg->data;
}

void target_yaw_degree_Callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_yaw_degree = msg->data;
}

void vision_cross_track_error_Callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    vision_cross_track_error = msg->data;
    control_action_flag = 1;
    // printf("vision XTE received!\n");
}

void vision_xte_offset_Callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    vision_xte_offset = msg->data;
}

void imu_yaw_degree_Callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    imu_heading_anlge_degree = msg->data;

    if (imu_heading_anlge_degree < 0)
    {
        imu_heading_anlge_degree += 360.0;
    }

    // ROS_INFO("imu_heading_anlge_degree : %6.3lf ",imu_heading_anlge_degree );

    control_action_flag = 1;
}

void steer_input_Callback(const std_msgs::msg::Int16::SharedPtr msg)
{
    steer_input = msg->data;
    control_action_flag = 1;
}

void yaw_control_mode_Callback(const std_msgs::msg::Int8::SharedPtr msg)
{
    steering_control_mode = msg->data;
    control_action_flag = 1;
}

void maze_xte_Callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    maze_xte = msg->data;
    control_action_flag = 1;
}

void slam_pose_Callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // Extract yaw from quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert to degrees
    slam_pose_yaw_degree = RAD2DEG(yaw);

    // Normalize to 0-360 range
    if (slam_pose_yaw_degree < 0)
    {
        slam_pose_yaw_degree += 360.0;
    }

    control_action_flag = 1;
}

void target_slam_yaw_Callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    target_slam_yaw_degree = msg->data;
}

double imu_control_angle_degree(void)
{
    double steering_angle_control = 0;

    error_imu_degree = target_yaw_degree - imu_heading_anlge_degree;

    if (error_imu_degree > 180)
    {
        error_imu_degree = error_imu_degree - 360;
    }
    else if (error_imu_degree < -180)
    {
        error_imu_degree = error_imu_degree + 360;
    }

    double error_imu_degree_d = error_imu_degree - error_imu_degree_old;

    steering_angle_control = Kp_imu_degree * error_imu_degree + Kd_imu_degree * error_imu_degree_d;

    error_imu_degree_old = error_imu_degree;

    return steering_angle_control;
}

double maze_control_yaw(void)
{
    double steering_angle_control = 0;

    double error_d = 0;

    error_maze = maze_xte;

    error_d = error_maze - error_maze_old;

    steering_angle_control = Kp_maze * error_maze + Kd_maze * error_d;

    error_maze_old = error_maze;

    if (steering_angle_control <= maze_right_angle_max)
        steering_angle_control = maze_right_angle_max;
    if (steering_angle_control >= maze_left_angle_max)
        steering_angle_control = maze_left_angle_max;

    printf("steering_control_mode : %d\n", steering_control_mode);
    printf("maze_xte : %6.3lf steer angle : %lf\n", maze_xte, steering_angle_control);

    return steering_angle_control;
}

double lidar_control_yaw(void)
{
    double steering_angle_control = 0;

    // Calculate error between target and current SLAM pose yaw
    error_lidar = target_slam_yaw_degree - slam_pose_yaw_degree;

    // Normalize error to -180 to 180 range
    if (error_lidar > 180)
    {
        error_lidar = error_lidar - 360;
    }
    else if (error_lidar < -180)
    {
        error_lidar = error_lidar + 360;
    }

    double error_d = error_lidar - error_lidar_old;

    steering_angle_control = Kp_lidar * error_lidar + Kd_lidar * error_d;

    error_lidar_old = error_lidar;

    // Limit steering angle
    if (steering_angle_control <= -42)
        steering_angle_control = -42;
    if (steering_angle_control >= 42)
        steering_angle_control = 42;

    printf("steering_control_mode : LIDAR_CONTROL\n");
    printf("target_yaw : %6.3lf current_yaw : %6.3lf error : %6.3lf steer angle : %lf\n",
           target_slam_yaw_degree, slam_pose_yaw_degree, error_lidar, steering_angle_control);

    return steering_angle_control;
}

double control_vision_xte(void)
{
    double steering_angle_control = 0;
    double error_vision_d = 0.0;
    // printf("sin(%6.3lf - %6.3lf) = %6.3lf\n ", target_yaw, yaw_d, CW_flag );
    // printf("error %6.3lf  %6.3lf \n", error1, error2);

    // Apply offset to vision XTE
    error_vision = vision_cross_track_error + vision_xte_offset;

    error_vision_d = error_vision - error_vision_old;

    steering_angle_control = Kp_vision * error_vision + Kd_vision * error_vision_d + Ki_vision * error_vision_sum;

    error_vision_old = error_vision;
    error_vision_sum = 0.0;

    printf("error : %6.3lf  erorr_d %6.3lf steering_angle :%6.3lf\n", error_vision, error_vision_d,
           steering_angle_control);

    if (steering_angle_control <= vision_xte_right_angle_max)
        steering_angle_control = vision_xte_right_angle_max;
    if (steering_angle_control >= vision_xte_left_angle_max)
        steering_angle_control = vision_xte_left_angle_max;

    return steering_angle_control;
}

int main(int argc, char** argv)
{
    double pid_output = 0.0;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("aa10_car_yaw_control_node");

    std::string imu_yaw_angle_topic = "/handsfree/imu_yaw_radian";
    std::string imu_angle_degree_topic = "/handsfree/imu_yaw_correction_degree";
    std::string yaw_control_mode_topic = "/Car_Control_Cmd/steering_control_mode";

    std::string yaw_target_topic = "/Car_Control_Cmd/Target_Angle";
    std::string yaw_control_steering_output_topic = "/Car_Control_Cmd/SteerAngle_Int16";
    std::string yaw_control_speed_input_topic = "/Car_Control_Cmd/Speed_Int16";

    std::string vision_cross_track_error_topic = "/xte/vision";
    std::string vision_xte_offset_topic = "/xte/vision_offset";
    std::string maze_xte_topic = "/xte/maze";
    std::string steer_input_topic = "/xte/steer";

    // Declare parameters
    node->declare_parameter("use_imu", use_imu);
    node->declare_parameter("imu_yaw_angle_topic", imu_yaw_angle_topic);
    node->declare_parameter("imu_angle_degree_topic", imu_angle_degree_topic);
    node->declare_parameter("yaw_control_mode_topic", yaw_control_mode_topic);
    node->declare_parameter("yaw_target_topic", yaw_target_topic);
    node->declare_parameter("steer_input_topic", steer_input_topic);
    node->declare_parameter("yaw_control_steering_output_topic", yaw_control_steering_output_topic);
    node->declare_parameter("vision_cross_track_error_topic", vision_cross_track_error_topic);
    node->declare_parameter("Kp_imu_degree", Kp_imu_degree);
    node->declare_parameter("Kd_imu_degree", Kd_imu_degree);
    node->declare_parameter("Ki_imu_degree", Ki_imu_degree);
    node->declare_parameter("Kp_vision", Kp_vision);
    node->declare_parameter("Kd_vision", Kd_vision);
    node->declare_parameter("Ki_vision", Ki_vision);
    node->declare_parameter("Kp_maze", Kp_maze);
    node->declare_parameter("Kd_maze", Kd_maze);
    node->declare_parameter("Ki_maze", Ki_maze);
    node->declare_parameter("Kp_lidar", Kp_lidar);
    node->declare_parameter("Kd_lidar", Kd_lidar);
    node->declare_parameter("Ki_lidar", Ki_lidar);
    node->declare_parameter("vision_xte_left_angle_max", vision_xte_left_angle_max);
    node->declare_parameter("vision_xte_right_angle_max", vision_xte_right_angle_max);
    node->declare_parameter("maze_left_angle_max", maze_left_angle_max);
    node->declare_parameter("maze_right_angle_max", maze_right_angle_max);

    // Get parameters
    node->get_parameter("use_imu", use_imu);
    node->get_parameter("imu_yaw_angle_topic", imu_yaw_angle_topic);
    node->get_parameter("imu_angle_degree_topic", imu_angle_degree_topic);
    node->get_parameter("yaw_control_mode_topic", yaw_control_mode_topic);
    node->get_parameter("yaw_target_topic", yaw_target_topic);
    node->get_parameter("steer_input_topic", steer_input_topic);
    node->get_parameter("yaw_control_steering_output_topic", yaw_control_steering_output_topic);
    node->get_parameter("vision_cross_track_error_topic", vision_cross_track_error_topic);
    node->get_parameter("Kp_imu_degree", Kp_imu_degree);
    node->get_parameter("Kd_imu_degree", Kd_imu_degree);
    node->get_parameter("Ki_imu_degree", Ki_imu_degree);
    node->get_parameter("Kp_vision", Kp_vision);
    node->get_parameter("Kd_vision", Kd_vision);
    node->get_parameter("Ki_vision", Ki_vision);
    node->get_parameter("Kp_maze", Kp_maze);
    node->get_parameter("Kd_maze", Kd_maze);
    node->get_parameter("Ki_maze", Ki_maze);
    node->get_parameter("Kp_lidar", Kp_lidar);
    node->get_parameter("Kd_lidar", Kd_lidar);
    node->get_parameter("Ki_lidar", Ki_lidar);
    node->get_parameter("vision_xte_left_angle_max", vision_xte_left_angle_max);
    node->get_parameter("vision_xte_right_angle_max", vision_xte_right_angle_max);
    node->get_parameter("maze_left_angle_max", maze_left_angle_max);
    node->get_parameter("maze_right_angle_max", maze_right_angle_max);

    roll = pitch = yaw = roll_d = pitch_d = yaw_d = yaw_d_old = 0.0;

    geometry_msgs::msg::Vector3 rpy_angle_radian;
    geometry_msgs::msg::Vector3 rpy_angle_degree;

    amap_powerpack_single_driver::msg::SteeringAngle steering_angle;

    auto sub_imu_yaw_degree =
        node->create_subscription<std_msgs::msg::Float32>(imu_angle_degree_topic, 1, imu_yaw_degree_Callback);
    auto sub_target_yaw_degree =
        node->create_subscription<std_msgs::msg::Float32>(yaw_target_topic, 1, target_yaw_degree_Callback);
    auto sub_yaw_control_mode =
        node->create_subscription<std_msgs::msg::Int8>(yaw_control_mode_topic, 1, yaw_control_mode_Callback);
    auto sub_vision_cross_track_error = node->create_subscription<std_msgs::msg::Float32>(
        vision_cross_track_error_topic, 1, vision_cross_track_error_Callback);
    auto sub_vision_xte_offset = node->create_subscription<std_msgs::msg::Float32>(
        vision_xte_offset_topic, 1, vision_xte_offset_Callback);
    auto sub_maze_xte = node->create_subscription<std_msgs::msg::Float32>(maze_xte_topic, 1, maze_xte_Callback);
    auto sub_car_speed_input = node->create_subscription<std_msgs::msg::Int16>(yaw_control_speed_input_topic, 2,
                                                                               yaw_control_speed_input_Callback);
    auto sub_steer_input = node->create_subscription<std_msgs::msg::Int16>(steer_input_topic, 2, steer_input_Callback);
    auto sub_slam_pose = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose", 1, slam_pose_Callback);
    auto sub_target_slam_yaw = node->create_subscription<std_msgs::msg::Float32>("/car_control/target_slam_yaw", 1, target_slam_yaw_Callback);

    auto yaw_control_steering_output_pub =
        node->create_publisher<amap_powerpack_single_driver::msg::SteeringAngle>(yaw_control_steering_output_topic, 1);

    rclcpp::Rate loop_rate(25.0); // 25.0HZ

    while (rclcpp::ok())
    {
        rpy_angle_radian.x = roll;
        rpy_angle_radian.y = pitch;
        rpy_angle_radian.z = yaw;

        rpy_angle_degree.x = roll_d;
        rpy_angle_degree.y = pitch_d;
        rpy_angle_degree.z = yaw_d;

        if (control_action_flag == 1)
        {
            if (steering_control_mode == IMU_CONTROL) // imu yaw control mode  2023.10.07
            {
                RCLCPP_INFO(node->get_logger(), "IMU_CONTROL_mode");

                pid_output = imu_control_angle_degree();

                // Check if target is reached using normalized error (handles 360 degree wraparound)
                if (fabs(error_imu_degree) <= 3)
                {
                    RCLCPP_INFO(node->get_logger(), "Target angle reached!");
                    steering_angle.steering_angle = 0; // 움직임을 멈춤
                    printf("steering_angle.steering_angle : %d\n", steering_angle.steering_angle);
                }
                else
                {
                    // Invert output for correct steering direction
                    // If speed is negative (reverse), invert steering again
                    if (car_speed < 0)
                    {
                        steering_angle.steering_angle = (int)(pid_output); // Reverse: don't invert
                        printf("REVERSE MODE - ");
                    }
                    else
                    {
                        steering_angle.steering_angle = (int)(-pid_output); // Forward: invert
                    }
                    printf("car_speed : %d\n", car_speed);
                    printf("target_yaw_degree : %6.3lf\n", target_yaw_degree);
                    printf("imu_heading_anlge_degree : %6.3lf\n", imu_heading_anlge_degree);
                    printf("error_imu_degree : %6.3lf\n", error_imu_degree);
                    printf("pid_output : %6.3lf\n", pid_output);
                    printf("steering_angle.steering_angle : %d\n", steering_angle.steering_angle);
                }

                yaw_control_steering_output_pub->publish(steering_angle);
            }

            else if (steering_control_mode == LANE_CONTROL) // vision control mode  2023.10.07
            {
                steering_angle.steering_angle = control_vision_xte(); // 부호 반전
                yaw_control_steering_output_pub->publish(steering_angle);
            }

            else if (steering_control_mode == MAZE_CONTROL) // traffic cone control
            {
                RCLCPP_INFO(node->get_logger(), "Maze_Control_mode");
                pid_output = maze_control_yaw();
                steering_angle.steering_angle = pid_output;
                yaw_control_steering_output_pub->publish(steering_angle);
                printf("steering_angle.steering_angle : %d\n", steering_angle.steering_angle);
            }

            else if (steering_control_mode == STEER_CONTROL)
            {
                RCLCPP_INFO(node->get_logger(), "Steer_Control_mode");
                steering_angle.steering_angle = steer_input;
                yaw_control_steering_output_pub->publish(steering_angle);
                printf("steering_angle.steering_angle : %d\n", steering_angle.steering_angle);
            }

            else if (steering_control_mode == LIDAR_CONTROL)
            {
                RCLCPP_INFO(node->get_logger(), "LIDAR_Control_mode");
                pid_output = lidar_control_yaw();
                steering_angle.steering_angle = pid_output;
                yaw_control_steering_output_pub->publish(steering_angle);
                printf("steering_angle.steering_angle : %d\n", steering_angle.steering_angle);
            }

            else
            {
            }
        }

        else
        {
            RCLCPP_WARN(node->get_logger(), "No target_angle");
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
