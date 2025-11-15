#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

#define WAYPOINT_CONTROL 4  // New control mode for waypoint following

class WaypointFollower : public rclcpp::Node
{
public:
    WaypointFollower() : Node("waypoint_follower")
    {
        // Declare parameters
        this->declare_parameter("lookahead_distance", 0.5);
        this->declare_parameter("max_linear_velocity", 0.3);
        this->declare_parameter("min_linear_velocity", 0.1);
        this->declare_parameter("max_angular_velocity", 1.0);
        this->declare_parameter("goal_tolerance", 0.15);
        this->declare_parameter("k_p", 1.0);
        this->declare_parameter("wheelbase", 0.2);

        // Get parameters
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_velocity").as_double();
        min_linear_vel_ = this->get_parameter("min_linear_velocity").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_velocity").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        k_p_ = this->get_parameter("k_p").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();

        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/sc_mini/sc_mini/cmd_vel", 10);

        // Publishers for yaw_control integration
        xte_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/xte/waypoint", 10);
        control_mode_pub_ = this->create_publisher<std_msgs::msg::Int8>(
            "/Car_Control_Cmd/steering_control_mode", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::Int16>(
            "/Car_Control_Cmd/Speed_Int16", 10);

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/waypoint_path", 10,
            std::bind(&WaypointFollower::pathCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&WaypointFollower::odomCallback, this, std::placeholders::_1));

        // Timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&WaypointFollower::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Waypoint Follower Node Started");
        RCLCPP_INFO(this->get_logger(), "Lookahead distance: %.2f m", lookahead_distance_);
        RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.2f m/s", max_linear_vel_);
    }

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty path");
            return;
        }

        path_ = *msg;
        current_waypoint_idx_ = 0;
        goal_reached_ = false;

        RCLCPP_INFO(this->get_logger(), "Received new path with %zu waypoints", path_.poses.size());
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;

        // Extract yaw from quaternion
        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
    }

    void controlLoop()
    {
        if (path_.poses.empty() || goal_reached_) {
            return;
        }

        // Find lookahead point
        geometry_msgs::msg::Point lookahead_point;
        bool found_lookahead = findLookaheadPoint(lookahead_point);

        if (!found_lookahead) {
            // No lookahead point found, check if goal is reached
            double dist_to_goal = distanceToGoal();
            if (dist_to_goal < goal_tolerance_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                goal_reached_ = true;
                stopRobot();
                return;
            }
            // Use the last waypoint as lookahead point
            lookahead_point = path_.poses.back().pose.position;
        }

        // Calculate steering angle using Pure Pursuit
        double steering_angle = calculateSteeringAngle(lookahead_point);

        // Publish velocity command
        publishVelocityCommand(steering_angle);
    }

    bool findLookaheadPoint(geometry_msgs::msg::Point& lookahead_point)
    {
        double min_dist_diff = std::numeric_limits<double>::max();
        int best_idx = -1;

        for (size_t i = current_waypoint_idx_; i < path_.poses.size(); ++i) {
            double dist = calculateDistance(
                current_pose_.position,
                path_.poses[i].pose.position);

            // Find point closest to lookahead distance
            double dist_diff = std::abs(dist - lookahead_distance_);

            if (dist >= lookahead_distance_ && dist_diff < min_dist_diff) {
                min_dist_diff = dist_diff;
                best_idx = i;
            }
        }

        if (best_idx >= 0) {
            lookahead_point = path_.poses[best_idx].pose.position;
            current_waypoint_idx_ = best_idx;
            return true;
        }

        return false;
    }

    double calculateSteeringAngle(const geometry_msgs::msg::Point& lookahead_point)
    {
        // Transform lookahead point to robot frame
        double dx = lookahead_point.x - current_pose_.position.x;
        double dy = lookahead_point.y - current_pose_.position.y;

        // Rotate to robot frame
        double x_robot = std::cos(current_yaw_) * dx + std::sin(current_yaw_) * dy;
        double y_robot = -std::sin(current_yaw_) * dx + std::cos(current_yaw_) * dy;

        // Calculate curvature using Pure Pursuit formula
        double ld_squared = x_robot * x_robot + y_robot * y_robot;
        double ld = std::sqrt(ld_squared);

        if (ld < 0.01) {
            return 0.0;
        }

        double curvature = 2.0 * y_robot / ld_squared;

        // Convert curvature to steering angle (Ackermann)
        double steering_angle = std::atan(curvature * wheelbase_);

        // Limit steering angle
        double max_steering = M_PI / 4; // 45 degrees
        steering_angle = std::clamp(steering_angle, -max_steering, max_steering);

        return steering_angle;
    }

    void publishVelocityCommand(double steering_angle)
    {
        // Publish control mode for yaw_control
        std_msgs::msg::Int8 control_mode;
        control_mode.data = WAYPOINT_CONTROL;
        control_mode_pub_->publish(control_mode);

        // Convert steering angle (radians) to cross-track error for yaw_control
        // Map steering angle to XTE range (approximation)
        double xte = steering_angle * 50.0; // Scale factor to convert to appropriate range
        xte = std::clamp(xte, -50.0, 50.0); // Limit XTE range

        std_msgs::msg::Float32 xte_msg;
        xte_msg.data = xte;
        xte_pub_->publish(xte_msg);

        // Calculate linear velocity (reduce speed for sharp turns)
        double abs_steering = std::abs(steering_angle);
        double vel_scale = 1.0 - (abs_steering / (M_PI / 4)) * 0.5;
        double linear_vel = max_linear_vel_ * vel_scale;
        linear_vel = std::max(linear_vel, min_linear_vel_);

        // Publish speed command (convert m/s to int16 scale)
        std_msgs::msg::Int16 speed_msg;
        speed_msg.data = static_cast<int16_t>(linear_vel * 100); // Scale for sc_mini
        speed_pub_->publish(speed_msg);

        RCLCPP_DEBUG(this->get_logger(),
            "XTE: %.2f, Speed: %d, Steering: %.2f deg",
            xte, speed_msg.data, steering_angle * 180.0 / M_PI);
    }

    void stopRobot()
    {
        // Stop via yaw_control
        std_msgs::msg::Int16 speed_msg;
        speed_msg.data = 0;
        speed_pub_->publish(speed_msg);

        std_msgs::msg::Float32 xte_msg;
        xte_msg.data = 0.0;
        xte_pub_->publish(xte_msg);

        // Also stop via cmd_vel as backup
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double distanceToGoal()
    {
        if (path_.poses.empty()) {
            return std::numeric_limits<double>::max();
        }
        return calculateDistance(current_pose_.position,
                                path_.poses.back().pose.position);
    }

    // Publishers and Subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr xte_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr control_mode_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr speed_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double lookahead_distance_;
    double max_linear_vel_;
    double min_linear_vel_;
    double max_angular_vel_;
    double goal_tolerance_;
    double k_p_;
    double wheelbase_;

    // State variables
    geometry_msgs::msg::Pose current_pose_;
    double current_yaw_ = 0.0;
    nav_msgs::msg::Path path_;
    size_t current_waypoint_idx_ = 0;
    bool goal_reached_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
