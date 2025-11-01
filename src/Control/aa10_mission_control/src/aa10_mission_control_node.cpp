#include "amap_powerpack_single_driver/msg/encoder_status.hpp"
#include "amap_powerpack_single_driver/msg/steering_angle.hpp"
#include "amap_powerpack_single_driver/msg/target_position_relative.hpp"
#include "amap_powerpack_single_driver/msg/target_speed.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <array>
#include <cmath>
#include <numeric>
#include <vector>

#define RAD2DEG(x) ((x) * 180. / M_PI)
#define DEG2RAD(x) ((x) / 180. * M_PI)

// Yaw Control Modes (must match aa10_car_yaw_control definitions)
#define IMU_CONTROL 0   // IMU-based yaw control
#define LANE_CONTROL 1  // Vision-based lane following (uses /xte/vision from ai_line_detection)
#define MAZE_CONTROL 2  // Maze navigation control
#define STEER_CONTROL 3 // Direct steering angle control
#define LIDAR_CONTROL 4 // LIDAR SLAM pose-based yaw control
#define CONE_CONTROL 5  // LiDAR cone following control (uses /xte/lidar_cone from aa10_lidar_cone_control)

// Additional mission control modes
#define STOP 99 // Stop mode (not used by yaw_control)
#define WALL_FOLLOWING 6
#define OBSTACLE_DETECT 7

// SLAM Pose structure
struct SlamPose
{
    double x;       // Position X in map frame (meters)
    double y;       // Position Y in map frame (meters)
    double yaw_rad; // Yaw angle in map frame (radians)

    SlamPose() : x(0.0), y(0.0), yaw_rad(0.0)
    {
    }
};

class MissionControlNode : public rclcpp::Node
{
  public:
    MissionControlNode() : Node("mission_control_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Parameters for real hardware
        this->declare_parameter("imu_topic", "/handsfree/imu_yaw_correction_degree");
        this->declare_parameter("lidar_topic", "/scan");
        this->declare_parameter("camera_topic", "/camera/image_raw");

        // Initialize variables
        imu_heading_angle_degree_ = 0.0;
        lidar_front_obstacle_distance_ = 0.0;
        stop_line_position_ = 0.0;
        wall_angle_correction_ = 0.0;
        mission_flag_ = 0;
        start_mission_flag_ = 0; // Default: start from mission 0
        end_mission_flag_ = 100; // Default: end at mission 100
        run_flag_ = false;
        lane_status_left_ = false;   // false = CLEAR, true = BLOCKED
        lane_status_center_ = false; // false = CLEAR, true = BLOCKED
        lane_status_right_ = false;  // false = CLEAR, true = BLOCKED
        // slam_pose_ initialized by struct constructor
        slam_pose_last_update_ = this->now();
        slam_pose_received_ = false;
        obstacle_distance_ = 0.0;
        obstacle_detected_ = false;
        lidar_cone_xte_ = 0.0;
        imu_angle_offset_ = 0.0;
        imu_calibrated_ = false;
        imu_offset_reset_sent_ = false;
        imu_calibration_samples_.clear();
        encoder_position_mm_ = 0.0;
        encoder_speed_mms_ = 0.0;
        encoder_reset_requested_ = false;
        mission4_saved_angle_ = 0.0;
        mission4_angle_saved_ = false;
        mission4_no_obstacle_count_ = 0;
        mission4_obstacle_count_ = 0;
        mission6_started_ = false;
        mission6_encoder_reset_sent_ = false;

        // Initialize mission arrays
        mission_start_position_mm_.fill(0.0);
        mission_start_slam_x_.fill(0.0);
        mission_start_slam_y_.fill(0.0);
        mission_position_sent_.fill(false);

        // Create subscribers for real hardware
        RCLCPP_INFO(this->get_logger(), "Starting in REAL HARDWARE mode");
        setupRealSubscribers();

        // Common subscribers (always available)
        run_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/flag/race_run", 10, std::bind(&MissionControlNode::runFlagCallback, this, std::placeholders::_1));

        stop_line_position_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/stop_line_position", 10,
            std::bind(&MissionControlNode::stopLinePositionCallback, this, std::placeholders::_1));

        lane_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/lane_status", 10, std::bind(&MissionControlNode::laneStatusCallback, this, std::placeholders::_1));

        obstacle_distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/obstacle/distance", 10,
            std::bind(&MissionControlNode::obstacleDistanceCallback, this, std::placeholders::_1));

        obstacle_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/obstacle/detected", 10,
            std::bind(&MissionControlNode::obstacleDetectedCallback, this, std::placeholders::_1));

        lidar_cone_xte_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/xte/lidar_cone", 10,
            std::bind(&MissionControlNode::lidarConeXteCallback, this, std::placeholders::_1));

        // SLAM Toolbox pose output location (requested at line 113)

        encoder_status_sub_ = this->create_subscription<amap_powerpack_single_driver::msg::EncoderStatus>(
            "/car_control/encoder_status", 10,
            std::bind(&MissionControlNode::encoderStatusCallback, this, std::placeholders::_1));

        // Mission control configuration subscribers
        start_mission_flag_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/mission_control/start_mission", 10,
            std::bind(&MissionControlNode::startMissionFlagCallback, this, std::placeholders::_1));

        end_mission_flag_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "/mission_control/end_mission", 10,
            std::bind(&MissionControlNode::endMissionFlagCallback, this, std::placeholders::_1));

        // Publishers
        yaw_control_mode_pub_ = this->create_publisher<std_msgs::msg::Int8>("/car_control/steering_control_mode", 5);
        lane_control_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/flag/lane_control_set", 5);
        target_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/car_control/target_angle", 5);
        target_angular_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("/target_angular_velocity", 5);
        obstacle_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_detect_enable", 5);

        // Create speed publishers for both modes
        car_control_speed_pub_ =
            this->create_publisher<amap_powerpack_single_driver::msg::TargetSpeed>("/car_control/target_speed", 5);

        // Create steering angle publisher (direct - may conflict with aa10_car_yaw_control)
        car_control_steering_pub_ =
            this->create_publisher<amap_powerpack_single_driver::msg::SteeringAngle>("/car_control/steering_angle", 5);

        // Create steer input publisher for STEER_CONTROL mode (via aa10_car_yaw_control)
        steer_input_pub_ = this->create_publisher<std_msgs::msg::Int16>("/xte/steer", 5);

        // Create vision XTE offset publisher
        vision_xte_offset_pub_ = this->create_publisher<std_msgs::msg::Float32>("/xte/vision_offset", 5);

        // Create position control publisher
        target_position_relative_pub_ =
            this->create_publisher<amap_powerpack_single_driver::msg::TargetPositionRelative>(
                "/car_control/target_position_relative", 5);

        // Create IMU offset publisher
        imu_offset_pub_ = this->create_publisher<std_msgs::msg::Float32>("/imu_angle_offset", 5);

        // Create mission status publisher for GUI
        mission_status_pub_ = this->create_publisher<std_msgs::msg::Int16>("/mission_control/mission_flag", 5);

        // Create SLAM command publisher
        slam_command_pub_ = this->create_publisher<std_msgs::msg::String>("/slam_command", 5);

        // Create target SLAM yaw publisher for LIDAR_CONTROL mode
        target_slam_yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/car_control/target_slam_yaw", 5);

        // Create mission target marker publisher for visualization
        mission_target_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/mission_target_marker", 5);

        // Timer for main control loop
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), // 10Hz
                                         std::bind(&MissionControlNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Mission Control Node initialized");
    }

    ~MissionControlNode()
    {
        // Stop the robot when node is shutting down
        RCLCPP_INFO(this->get_logger(), "Shutting down - stopping vehicle and SLAM Toolbox");

        // Send STOP mode to aa10_car_yaw_control
        publishControlMode(STOP);

        // Send zero speed
        publishSpeedReal(0);

        // Also disable lane control
        publishLaneControl(false);

        // Stop SLAM Toolbox
        slamToolboxStop();

        // Give time for messages to be sent
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

  private:
    void setupRealSubscribers()
    {
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        std::string camera_topic = this->get_parameter("camera_topic").as_string();

        // IMU subscriber for real hardware (Float32 yaw degree)
        imu_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            imu_topic, 10, std::bind(&MissionControlNode::imuCallback, this, std::placeholders::_1));

        // Lidar subscriber for real hardware (sensor_msgs/LaserScan)
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&MissionControlNode::lidarCallback, this, std::placeholders::_1));

        // Camera subscriber for real hardware (sensor_msgs/Image)
        camera_real_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10, std::bind(&MissionControlNode::cameraRealCallback, this, std::placeholders::_1));

        // SLAM Toolbox pose subscriber (geometry_msgs/PoseWithCovarianceStamped)
        slam_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pose", 10, std::bind(&MissionControlNode::slamPoseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Real hardware subscribers created");
        RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Lidar topic: %s", lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "SLAM Pose topic: /pose");
    }

    // Real hardware callbacks
    void imuCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // Receive corrected IMU value from IMU node (after offset applied)
        imu_heading_angle_degree_ = msg->data;
        if (imu_heading_angle_degree_ < 0)
        {
            imu_heading_angle_degree_ += 360.0;
        }
        RCLCPP_DEBUG(this->get_logger(), "IMU yaw (corrected): %.1f°", imu_heading_angle_degree_);
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Extract front distance from LaserScan (front beam at index 0)
        if (!msg->ranges.empty())
        {
            size_t front_index = 0; // 0도 = 전방
            lidar_front_obstacle_distance_ = msg->ranges[front_index];

            // Handle inf/nan values
            if (std::isinf(lidar_front_obstacle_distance_) || std::isnan(lidar_front_obstacle_distance_))
            {
                lidar_front_obstacle_distance_ = msg->range_max;
            }

            // Store all ranges and scan parameters for wall detection
            lidar_ranges_ = msg->ranges;
            lidar_angle_min_ = msg->angle_min;
            lidar_angle_increment_ = msg->angle_increment;
        }
    }

    void cameraRealCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Camera image received - for now just log
        static int camera_count = 0;
        if (++camera_count % 30 == 0)
        { // Log every 30 frames (~1 second at 30Hz)
            RCLCPP_DEBUG(this->get_logger(), "Real Camera image received: %dx%d, encoding: %s", msg->width, msg->height,
                         msg->encoding.c_str());
        }
    }

    void slamPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // SLAM Toolbox pose (map frame)
        slam_pose_.x = msg->pose.pose.position.x;
        slam_pose_.y = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        slam_pose_.yaw_rad = yaw;

        // Update last received time
        slam_pose_last_update_ = this->now();
        slam_pose_received_ = true;

        // Output SLAM pose in map frame (throttled to 1Hz) with mission number
        // Mission 100에서는 출력하지 않음
        if (mission_flag_ != 100)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "\n========================================\n"
                                 "Mission %d | SLAM Pose (map frame)\n"
                                 "  Position: x=%.3f m, y=%.3f m\n"
                                 "  Orientation: yaw=%.3f° (%.3f rad)\n"
                                 "========================================",
                                 mission_flag_, slam_pose_.x, slam_pose_.y, RAD2DEG(slam_pose_.yaw_rad),
                                 slam_pose_.yaw_rad);

            // Transform to base_link frame and output (throttled to 1Hz)
            try
            {
                // Get transform from map to base_link
                geometry_msgs::msg::TransformStamped transform_stamped =
                    tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero);

                // Create a point in map frame (current SLAM pose)
                geometry_msgs::msg::PointStamped point_map;
                point_map.header.frame_id = "map";
                point_map.header.stamp = this->now();
                point_map.point.x = slam_pose_.x;
                point_map.point.y = slam_pose_.y;
                point_map.point.z = 0.0;

                // Transform to base_link frame
                geometry_msgs::msg::PointStamped point_base_link;
                tf2::doTransform(point_map, point_base_link, transform_stamped);

                // Extract yaw in base_link frame
                tf2::Quaternion q_base(transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
                                       transform_stamped.transform.rotation.z,
                                       transform_stamped.transform.rotation.w);
                tf2::Matrix3x3 m_base(q_base);
                double roll_base, pitch_base, yaw_base;
                m_base.getRPY(roll_base, pitch_base, yaw_base);

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Mission %d | SLAM Pose (base_link frame)\n"
                                     "  Position: x=%.3f m, y=%.3f m\n"
                                     "  Orientation: yaw=%.3f° (%.3f rad)",
                                     mission_flag_, point_base_link.point.x, point_base_link.point.y, RAD2DEG(yaw_base),
                                     yaw_base);
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_DEBUG(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
            }
        }
    }

    void runFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        run_flag_ = msg->data;

        // When race starts, set mission_flag to start_mission_flag_
        if (run_flag_)
        {
            mission_flag_ = start_mission_flag_;
            RCLCPP_INFO(this->get_logger(), "Race started - Starting from Mission %d (end at Mission %d)",
                        start_mission_flag_, end_mission_flag_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Race stopped");
        }
    }

    void startMissionFlagCallback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        start_mission_flag_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Start mission flag set to: %d", start_mission_flag_);
    }

    void endMissionFlagCallback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        end_mission_flag_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "End mission flag set to: %d", end_mission_flag_);
    }

    void stopLinePositionCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        stop_line_position_ = msg->data;
        // 특정 mission_flag에서만 로그 출력 (case 1, 3, 5, 16 등 stop line이 필요한 경우)
        if (mission_flag_ == 1 || mission_flag_ == 3 || mission_flag_ == 5 || mission_flag_ == 16)
        {
            RCLCPP_INFO(this->get_logger(), "Stop line position received: %.3f", stop_line_position_);
        }
    }

    void laneStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Parse the lane status string: "LEFT,RIGHT" or "LEFT,CENTER,RIGHT"
        std::string status_str = msg->data;
        size_t first_comma = status_str.find(',');

        if (first_comma != std::string::npos)
        {
            std::string left_str = status_str.substr(0, first_comma);
            size_t second_comma = status_str.find(',', first_comma + 1);

            if (second_comma != std::string::npos)
            {
                // Format: "LEFT,CENTER,RIGHT"
                std::string center_str = status_str.substr(first_comma + 1, second_comma - first_comma - 1);
                std::string right_str = status_str.substr(second_comma + 1);

                lane_status_left_ = (left_str == "DETECTED");
                lane_status_center_ = (center_str == "DETECTED");
                lane_status_right_ = (right_str == "DETECTED");
            }
            else
            {
                // Format: "LEFT,RIGHT" (new format from ai_line_detection)
                std::string right_str = status_str.substr(first_comma + 1);

                lane_status_left_ = (left_str == "DETECTED");
                lane_status_right_ = (right_str == "DETECTED");
                lane_status_center_ = false;  // Not used in new format
            }

            // 로그 출력 (Mission 27에서 활성화)
            if (mission_flag_ == 27)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Lane Status: Left=%s, Right=%s",
                    lane_status_left_ ? "DETECTED" : "CLEAR",
                    lane_status_right_ ? "DETECTED" : "CLEAR");
            }
        }
    }

    void obstacleDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        obstacle_distance_ = msg->data;
        // Mission 0에서만 로그 출력 (가림막 감지)
        if (mission_flag_ == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Obstacle distance: %.3f m", obstacle_distance_);
        }
    }

    void obstacleDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        obstacle_detected_ = msg->data;
        // Mission 0에서만 로그 출력 (가림막 감지)
        if (mission_flag_ == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Obstacle detected: %s", obstacle_detected_ ? "true" : "false");
        }
    }

    void lidarConeXteCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        lidar_cone_xte_ = msg->data;
        // Mission 31에서만 로그 출력 (콘 추종)
        if (mission_flag_ == 31)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Lidar Cone XTE: %.3f", lidar_cone_xte_);
        }
    }

    void encoderStatusCallback(const amap_powerpack_single_driver::msg::EncoderStatus::SharedPtr msg)
    {
        encoder_position_mm_ = msg->position_mm;
        encoder_speed_mms_ = msg->speed_mms;
        RCLCPP_DEBUG(this->get_logger(), "Encoder: position=%.1f mm, speed=%.1f mm/s", encoder_position_mm_,
                     encoder_speed_mms_);
    }

    void controlLoop()
    {
        auto current_time = this->now();

        if (run_flag_)
        {
            // Main mission control logic - 상태 로그 출력 (Mission 100 제외)
            if (mission_flag_ != 100)
            {
                double current_speed = 100.0; // Current speed based on mode
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Mission: %d | IMU: %.1f° | Speed: %.2f", mission_flag_, imu_heading_angle_degree_,
                                     current_speed);
            }

            // Simple mission example
            switch (mission_flag_)
            {
            case 0:
                // Initialize mission - 출발 대기 (가림막 제거 대기)
                publishLaneControl(false);
                publishControlMode(STEER_CONTROL); // Direct steering control mode
                publishSteerInput(0);              // Use /xte/steer for STEER_CONTROL mode
                publishSpeedReal(0);
                publishVisionXteOffset(0.0); // Initialize vision offset to 0
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 0: Waiting for barrier removal...");

                // SLAM Toolbox 노드 종료 (한 번만 호출)
                static bool slam_killed = false;
                if (!slam_killed)
                {
                    system("pkill -9 -f 'async_slam_toolbox_node' &");
                    slam_killed = true;
                    RCLCPP_INFO(this->get_logger(), "SLAM Toolbox node killed for initialization");
                }

                // Encoder 리셋 (한 번만 호출)
                if (!encoder_reset_requested_)
                {
                    // ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger 명령 실행
                    system("ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger &");
                    RCLCPP_INFO(this->get_logger(), "Encoder reset requested");
                    encoder_reset_requested_ = true;
                }

                // 가림막이 제거되면 mission_flag를 1로 변경
                // Obstacle detect 노드로부터 거리 정보 받아서 판단
                // obstacle_detected_ == false: 장애물 없음 (가림막 제거됨)
                // 또는 obstacle_distance_ > 임계값: 가림막이 충분히 멀어짐
                if (!obstacle_detected_ || obstacle_distance_ > 0.5)
                {
                    // 가림막이 없거나 50cm 이상 멀어지면 출발
                    RCLCPP_INFO(this->get_logger(), "Barrier removed (detected=%s, distance=%.2fm)",
                                obstacle_detected_ ? "true" : "false", obstacle_distance_);
                    transitionToNextMission(1); // Move to mission 1
                }
                else
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Waiting for barrier removal... (distance=%.2fm)", obstacle_distance_);
                }
                break;

            case 1:
                // AI 라인 검출 주행
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 1: Lane following with AI line detection");

                // Lane control 활성화
                publishLaneControl(true);
                publishControlMode(LANE_CONTROL); // Vision control mode
                publishSpeedReal(280);            // 주행 속도 설정
                publishVisionXteOffset(0.0);      // Vision offset 0 (중앙)

                // Stop line 검출 확인
                if (stop_line_position_ > 0.0)
                {
                    // Y 좌표가 클수록 차에 가까움 (이미지 하단)
                    // 640x100 이미지 기준:
                    //   Y = 0~40: 멀리 (상단)
                    //   Y > 40: 가까움 (중간~하단) → 정지
                    if (stop_line_position_ > 40.0)
                    {
                        RCLCPP_INFO(this->get_logger(), "Stop line detected close (y=%.1f px) - Stopping!",
                                    stop_line_position_);
                        publishVisionXteOffset(0.0); // Reset vision offset
                        transitionToNextMission(2);
                    }
                    else
                    {
                        RCLCPP_DEBUG(this->get_logger(), "Stop line detected but far (y=%.1f px) - Continue driving",
                                     stop_line_position_);
                    }
                }
                break;

            case 2:
                // 정지선에서 정지 후 Mission 3으로 전환
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 2: Stopped at stop line");

                publishLaneControl(false);
                publishControlMode(STOP);
                publishSpeedReal(0);

                // 짧은 정지 후 Mission 3으로 전환 (0.5초 대기)
                static auto mission2_stop_time = this->now();
                static bool mission2_stopped = false;

                if (!mission2_stopped)
                {
                    mission2_stop_time = this->now();
                    mission2_stopped = true;
                }

                if ((this->now() - mission2_stop_time).seconds() > 1.0)
                {
                    mission2_stopped = false; // Reset for next time
                    RCLCPP_INFO(this->get_logger(), "Mission 2 complete - Moving to next");
                    transitionToNextMission(3);
                }
                break;

            case 3:
            {
                // 0.05m/s (50mm/s) 속도로 10cm 전진 후 정지, SLAM Toolbox 시작
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 3: Moving 10cm forward at 50 mm/s, then start SLAM");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL); // Direct steering control mode
                publishSteerInput(0);              // Use /xte/steer for STEER_CONTROL mode (straight)

                // 시작 위치 저장 (한 번만)
                if (!mission_position_sent_[3])
                {
                    mission_start_position_mm_[3] = encoder_position_mm_;
                    mission_position_sent_[3] = true;

                    RCLCPP_INFO(this->get_logger(), "Mission 3 started at position: %.1fmm", encoder_position_mm_);
                }

                // 현재까지 이동한 거리
                double traveled_distance = encoder_position_mm_ - mission_start_position_mm_[3];

                // 목표: 100mm (10cm) 이동 (관성 보정: 95mm에서 정지 명령)
                if (traveled_distance < 300.0)
                {
                    // 아직 목표 거리에 도달하지 않음 - 속도 50mm/s로 전진
                    publishSpeedReal(130); // 50mm/s = 0.05m/s

                    // 진행 상황 모니터링
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 500,
                        "Mission 3: Traveled %.1f/95.0 mm (target: 100mm with overshoot), speed: %.1f mm/s",
                        traveled_distance, encoder_speed_mms_);
                }
                else
                {
                    // 목표 거리 도달 - 정지 후 SLAM 시작
                    publishSpeedReal(0);

                    // SLAM 시작 명령 전송 (한 번만)
                    static bool slam_command_sent = false;
                    static rclcpp::Time slam_start_time = this->now();
                    static rclcpp::Time slam_check_start_time = this->now();

                    if (!slam_command_sent)
                    {
                        // SLAM 시작 전 pose 수신 플래그 초기화
                        slam_pose_received_ = false;

                        // SLAM Toolbox 노드를 launch (백그라운드에서 실행)
                        system("ros2 launch slam_toolbox_config online_async_launch.py > /dev/null 2>&1 &");
                        slam_start_time = this->now();
                        slam_check_start_time = this->now();
                        slam_command_sent = true;

                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 3: Stopped at %.1fmm. SLAM Toolbox node launched.",
                                    encoder_position_mm_);
                    }

                    auto elapsed_time = (this->now() - slam_start_time).seconds();
                    auto time_since_last_pose = (this->now() - slam_pose_last_update_).seconds();

                    // SLAM 실행 확인: pose가 수신되고 있는지 확인 (최근 1초 이내)
                    bool slam_running = slam_pose_received_ && (time_since_last_pose < 1.0);

                    // SLAM 시작 후 3초 대기하고 SLAM이 실행 중임을 확인 (노드 launch 시간 필요)
                    if (slam_running && elapsed_time >= 3.0)
                    {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Mission 3 complete: SLAM Toolbox node running confirmed (%.1fs elapsed, pose: x=%.3f, y=%.3f)",
                            elapsed_time, slam_pose_.x, slam_pose_.y);

                        // 다음 미션으로 전환하기 전 플래그 리셋
                        mission_position_sent_[3] = false;
                        slam_command_sent = false;

                        transitionToNextMission(4);
                    }
                    else if (elapsed_time >= 5.0)
                    {
                        // 5초 경과했는데도 SLAM pose가 수신되지 않으면 경고하고 진행
                        RCLCPP_WARN(this->get_logger(),
                                    "Mission 3: SLAM pose not received after %.1fs - proceeding anyway", elapsed_time);

                        mission_position_sent_[3] = false;
                        slam_command_sent = false;

                        transitionToNextMission(4);
                    }
                    else
                    {
                        if (slam_running)
                        {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                                 "Mission 3: SLAM running, waiting... %.1fs / 1.0s", elapsed_time);
                        }
                        else
                        {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                                 "Mission 3: Waiting for SLAM to start... %.1fs (max 3.0s)",
                                                 elapsed_time);
                        }
                    }
                }
                break;
            }

            case 4:
            {
                // 장애물 감지 - 현재 각도 저장하고 장애물 유무에 따라 분기
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 4: Checking obstacle and saving current angle");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);
                publishSteerInput(0); // Use /xte/steer for STEER_CONTROL mode
                publishSpeedReal(0);  // 정지 상태에서 판단

                // 현재 IMU 각도 저장 (한 번만)
                if (!mission4_angle_saved_)
                {
                    mission4_saved_angle_ = imu_heading_angle_degree_;
                    mission4_angle_saved_ = true;
                    RCLCPP_INFO(this->get_logger(), "Mission 4: Saved reference angle = %.1f°", mission4_saved_angle_);
                }

                // 장애물 감지 여부 카운팅 (연속 검출 확인)
                const int REQUIRED_COUNT = 5; // 연속 5회 검출 필요

                if (!obstacle_detected_ || obstacle_distance_ > 2.0)
                {
                    // 장애물 없음으로 판단
                    mission4_no_obstacle_count_++;
                    mission4_obstacle_count_ = 0; // 장애물 카운트 리셋

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                         "Mission 4: No obstacle (%d/%d) - distance=%.2fm", mission4_no_obstacle_count_,
                                         REQUIRED_COUNT, obstacle_distance_);

                    if (mission4_no_obstacle_count_ >= REQUIRED_COUNT)
                    {
                        // 연속 5회 이상 "장애물 없음" 확정 -> Mission 5 (직진 30cm)
                        mission4_angle_saved_ = false;   // Reset for next time
                        mission4_no_obstacle_count_ = 0; // Reset counter
                        mission4_obstacle_count_ = 0;    // Reset counter
                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 4: No obstacle confirmed (distance=%.2fm) - Going straight 30cm",
                                    obstacle_distance_);
                        transitionToNextMission(5);
                    }
                }
                else
                {
                    // 장애물 있음으로 판단
                    mission4_obstacle_count_++;
                    mission4_no_obstacle_count_ = 0; // 장애물 없음 카운트 리셋

                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                         "Mission 4: Obstacle detected (%d/%d) - distance=%.2fm",
                                         mission4_obstacle_count_, REQUIRED_COUNT, obstacle_distance_);

                    if (mission4_obstacle_count_ >= REQUIRED_COUNT)
                    {
                        // 연속 5회 이상 "장애물 있음" 확정 -> Mission 6 (yaw -30도로 회피하며 30cm)
                        mission4_angle_saved_ = false;   // Reset for next time
                        mission4_no_obstacle_count_ = 0; // Reset counter
                        mission4_obstacle_count_ = 0;    // Reset counter
                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 4: Obstacle confirmed (distance=%.2fm) - Avoiding with yaw -30°",
                                    obstacle_distance_);
                        transitionToNextMission(10); // 장애물 있으면 10번 우회전으로 코스 선택
                    }
                }
                break;
            }

            case 5:
            {
                // 비전 레인 제어로 1.4m 주행 후 정지
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 5: Lane following with vision control for 1.4m");

                publishLaneControl(true);
                publishControlMode(LANE_CONTROL); // Vision-based lane control
                publishVisionXteOffset(0.0);      // Vision offset 0 (중앙)

                // 시작 위치 저장 (한 번만)
                if (!mission_position_sent_[5])
                {
                    mission_start_position_mm_[5] = encoder_position_mm_;
                    mission_position_sent_[5] = true;
                    RCLCPP_INFO(this->get_logger(), "Mission 5 started at position: %.1fmm with vision lane control",
                                encoder_position_mm_);
                }

                // 현재까지 이동한 거리
                double traveled_distance = encoder_position_mm_ - mission_start_position_mm_[5];

                // 목표: 1400mm (1.4m) 이동
                if (traveled_distance < 1400.0)
                {
                    publishSpeedReal(350); // 주행 속도 설정
                }
                else
                {
                    publishSpeedReal(0);
                    publishLaneControl(false);
                    publishControlMode(STOP);
                    publishVisionXteOffset(0.0); // Reset vision offset
                    RCLCPP_INFO(this->get_logger(), "Mission 5 complete: Traveled %.1fmm with lane control",
                                traveled_distance);
                    mission_position_sent_[5] = false;
                    transitionToNextMission(6); // Mission 6으로 전환 (정지 및 엔코더 리셋)
                }

                // 진행 상황 모니터링
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Mission 5: Traveled %.1f/1400.0 mm with vision lane control", traveled_distance);
                break;
            }

            case 6:
            {
                // 엔코더 리셋 확인 및 재시도
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 6: Resetting encoder until zero");

                publishLaneControl(false);
                publishControlMode(STOP);
                publishSpeedReal(0);

                // Mission 6 시작 시간 기록
                if (!mission6_started_)
                {
                    mission6_start_time_ = this->now();
                    mission6_started_ = true;
                    mission6_encoder_reset_sent_ = false;
                    RCLCPP_INFO(this->get_logger(), "Mission 6: Started encoder reset process");
                }

                double elapsed_time = (this->now() - mission6_start_time_).seconds();

                // 0.2초마다 엔코더 값 확인
                if (elapsed_time >= 0.2)
                {
                    // 엔코더 값이 0이 아니면 리셋 (절대값 50mm 이상)
                    if (std::abs(encoder_position_mm_) > 50.0)
                    {
                        system("ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger &");
                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 6: Encoder not zero (%.1fmm) - Resetting again at %.2fs",
                                    encoder_position_mm_, elapsed_time);
                        mission6_start_time_ = this->now(); // 타이머 리셋
                        elapsed_time = 0.0;
                    }
                    else
                    {
                        // 엔코더 값이 0에 가까우면 Mission 7로 전환
                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 6: Encoder reset confirmed (%.1fmm) - Transitioning to Mission 7",
                                    encoder_position_mm_);
                        mission6_started_ = false;
                        mission6_encoder_reset_sent_ = false;
                        transitionToNextMission(7);
                    }
                }
                else
                {
                    // 첫 리셋 요청
                    if (!mission6_encoder_reset_sent_)
                    {
                        system("ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger &");
                        mission6_encoder_reset_sent_ = true;
                        RCLCPP_INFO(this->get_logger(), "Mission 6: Initial encoder reset requested");
                    }
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                         "Mission 6: Waiting for encoder update... %.2fs (current=%.1fmm)",
                                         elapsed_time, encoder_position_mm_);
                }
                break;
            }

            case 7:
            {
                // 비전 레인 제어로 2m 주행 후 정지
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 7: Lane following with vision control for 2m");

                publishLaneControl(true);
                publishControlMode(LANE_CONTROL); // Vision-based lane control

                // 시작 위치 저장 (한 번만)
                if (!mission_position_sent_[7])
                {
                    mission_start_position_mm_[7] = encoder_position_mm_;
                    mission_position_sent_[7] = true;
                    RCLCPP_INFO(this->get_logger(), "Mission 7 started at position: %.1fmm", encoder_position_mm_);
                }

                // 현재까지 이동한 거리
                double traveled_distance = encoder_position_mm_ - mission_start_position_mm_[7];

                // 목표: 2000mm (2m) 이동
                if (traveled_distance < 2600.0)
                {
                    publishSpeedReal(330); // 주행 속도 설정
                }
                else
                {
                    // 목표 거리 도달 - 정지
                    publishSpeedReal(0);
                    publishLaneControl(false);
                    publishControlMode(STOP);
                    publishVisionXteOffset(0.0); // Reset vision offset

                    RCLCPP_INFO(this->get_logger(), "Mission 7 complete: Traveled %.1fmm with lane control",
                                traveled_distance);
                    mission_position_sent_[7] = false; // Reset for next time
                    transitionToNextMission(8);        // Mission 100로 전환
                }

                // 진행 상황 모니터링
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Mission 7: Traveled %.1f/2000.0 mm with vision lane control", traveled_distance);
                break;
            }

            case 8:
            {
                // 미션 8: 조향각 -20도로 1000mm 주행 후 정지
                executeSteerDistanceMission(8, -20, 1000.0, 200, 100);
                break;
            }

            case 10:
            {
                // 미션 10: SLAM pose 기준 목표 위치(0.860, -0.985)로 이동, base_link 기준 판단
                RCLCPP_INFO_ONCE(this->get_logger(),
                                 "Mission 10: Moving to target SLAM pose (x=0.860m, y=-0.985m) using base_link frame");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL); // Direct steering control mode
                publishSteerInput(19);             // 직진 조향

                // 목표 위치 (map frame)
                const double target_x = 0.713;          // meters
                const double target_y = -0.92;          // meters
                const double position_tolerance = 0.10; // 10cm = 0.1m

                // Publish target marker (green sphere at target location)
                static bool marker_published = false;
                if (!marker_published) {
                    publishMissionTargetMarker(target_x, target_y, 10, "green");
                    marker_published = true;
                }

                // Map frame에서의 거리 계산
                double dx_map = slam_pose_.x - target_x;
                double dy_map = slam_pose_.y - target_y;
                double distance_to_target = std::sqrt(dx_map * dx_map + dy_map * dy_map);

                // 목표를 base_link 좌표계로 변환
                bool target_reached = false;
                double target_x_baselink = 0.0;
                double target_y_baselink = 0.0;

                if (transformPointToBaseLink(target_x, target_y, target_x_baselink, target_y_baselink))
                {
                    // 판단 로직: 목표가 뒤에 있거나(x < 0) 충분히 가까우면 정지
                    if (target_x_baselink < -position_tolerance)
                    {
                        // 목표를 지나침 (목표가 뒤에 있음)
                        target_reached = true;
                        RCLCPP_INFO(this->get_logger(), "Mission 10: Target passed (behind vehicle)");
                    }
                    else if (target_x_baselink >= -position_tolerance && target_x_baselink <= position_tolerance &&
                             std::fabs(target_y_baselink) <= position_tolerance)
                    {
                        // 목표가 tolerance 범위 내에 있음
                        target_reached = true;
                        RCLCPP_INFO(this->get_logger(), "Mission 10: Target reached (within tolerance)");
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Mission 10: Could not transform target to base_link");
                }

                if (!target_reached)
                {
                    // 아직 목표에 도달하지 않음 - 계속 전진
                    publishSpeedReal(280); // 280mm/s

                    // 진행 상황 모니터링
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 500,
                        "Mission 10: Target(map): (%.3f, %.3f) | Current(map): (%.3f, %.3f) | Dist: %.3fm | "
                        "Target(base_link): (%.3f, %.3f)",
                        target_x, target_y, slam_pose_.x, slam_pose_.y, distance_to_target, target_x_baselink,
                        target_y_baselink);
                }
                else
                {
                    // 목표 위치 도달 - 정지
                    publishSteerInput(0);
                    publishSpeedReal(0);

                    // Delete target marker
                    deleteMissionTargetMarker(10);
                    marker_published = false;  // Reset for next time mission 10 runs

                    RCLCPP_INFO(this->get_logger(), "Mission 10 complete!");
                    RCLCPP_INFO(this->get_logger(), "Target(map): (%.3f, %.3f) | Current(map): (%.3f, %.3f)", target_x,
                                target_y, slam_pose_.x, slam_pose_.y);
                    RCLCPP_INFO(this->get_logger(), "Target(base_link): x=%.3fm, y=%.3fm", target_x_baselink,
                                target_y_baselink);
                    RCLCPP_INFO(this->get_logger(), "Map distance: %.3fm", distance_to_target);

                    mission_position_sent_[10] = false; // Reset for next time
                    transitionToNextMission(11);
                }
                break;
            }

            case 11:
            {
                // 미션 11: SLAM pose 기준 목표 위치(1.531, -1.727)로 이동, base_link 기준 판단
                RCLCPP_INFO_ONCE(this->get_logger(),
                                 "Mission 11: Moving to target SLAM pose (x=1.531m, y=-1.727m) using base_link frame");

                publishLaneControl(true);
                publishControlMode(LANE_CONTROL); // Vision-based lane control
                publishSpeedReal(150);            // 주행 속도 설정
                publishVisionXteOffset(-50);    // Vision offset -50 (왼쪽으로 치우치게)

                // 목표 위치 (map frame) x=1.320 m, y=-2.102 m

                const double target_x = 1.320;          // meters
                const double target_y = -2.0;          // meters
                const double position_tolerance = 0.1; // 10cm = 0.1m

                // Map frame에서의 거리 계산
                double dx_map = slam_pose_.x - target_x;
                double dy_map = slam_pose_.y - target_y;
                double distance_to_target = std::sqrt(dx_map * dx_map + dy_map * dy_map);

                // 목표를 base_link 좌표계로 변환
                bool target_reached = false;
                double target_x_baselink = 0.0;
                double target_y_baselink = 0.0;

                if (transformPointToBaseLink(target_x, target_y, target_x_baselink, target_y_baselink))
                {
                    // 판단 로직: 목표가 뒤에 있거나(x < 0) 충분히 가까우면 정지
                    if (target_x_baselink < -position_tolerance)
                    {
                        // 목표를 지나침 (목표가 뒤에 있음)
                        target_reached = true;
                        RCLCPP_INFO(this->get_logger(), "Mission 11: Target passed (behind vehicle)");
                    }
                    else if (target_x_baselink >= -position_tolerance && target_x_baselink <= position_tolerance &&
                             std::fabs(target_y_baselink) <= position_tolerance)
                    {
                        // 목표가 tolerance 범위 내에 있음
                        target_reached = true;
                        RCLCPP_INFO(this->get_logger(), "Mission 11: Target reached (within tolerance)");
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Mission 11: Could not transform target to base_link");
                }

                if (!target_reached)
                {
                    // 아직 목표에 도달하지 않음 - 계속 전진
                    publishSpeedReal(160); // 350mm/s

                    // 진행 상황 모니터링
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 500,
                        "Mission 11: Target(map): (%.3f, %.3f) | Current(map): (%.3f, %.3f) | Dist: %.3fm | "
                        "Target(base_link): (%.3f, %.3f)",
                        target_x, target_y, slam_pose_.x, slam_pose_.y, distance_to_target, target_x_baselink,
                        target_y_baselink);
                }
                else
                {
                    // 목표 위치 도달 - 정지
                    publishSpeedReal(0);
                    publishLaneControl(false);
                    publishControlMode(STOP);
                    publishVisionXteOffset(0.0); // Reset vision offset

                    RCLCPP_INFO(this->get_logger(), "Mission 11 complete!");
                    RCLCPP_INFO(this->get_logger(), "Target(map): (%.3f, %.3f) | Current(map): (%.3f, %.3f)", target_x,
                                target_y, slam_pose_.x, slam_pose_.y);
                    RCLCPP_INFO(this->get_logger(), "Target(base_link): x=%.3fm, y=%.3fm", target_x_baselink,
                                target_y_baselink);
                    RCLCPP_INFO(this->get_logger(), "Map distance: %.3fm", distance_to_target);

                    mission_position_sent_[11] = false; // Reset for next time
                    transitionToNextMission(12);        // Mission 100으로 전환
                }
                break;
            }

            case 12:
            {
                // 미션 12: SLAM pose 기준 목표 위치(x=1.489, y=-2.536)로 이동, base_link 기준 판단
                RCLCPP_INFO_ONCE(this->get_logger(),
                                 "Mission 12: Moving to target SLAM pose (x=1.489m, y=-2.536m) using base_link frame");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL); // Direct steering control mode
                publishSteerInput(14);             // 조향각 15도

                // 목표 위치 (map frame)
                const double target_x = 1.489;          // meters
                const double target_y = -2.500;         // meters
                const double position_tolerance = 0.10; // 10cm = 0.1m

                // Map frame에서의 거리 계산
                double dx_map = slam_pose_.x - target_x;
                double dy_map = slam_pose_.y - target_y;
                double distance_to_target = std::sqrt(dx_map * dx_map + dy_map * dy_map);

                // 목표를 base_link 좌표계로 변환
                bool target_reached = false;
                double target_x_baselink = 0.0;
                double target_y_baselink = 0.0;

                if (transformPointToBaseLink(target_x, target_y, target_x_baselink, target_y_baselink))
                {
                    // 판단 로직: 목표가 뒤에 있거나(x < 0) 충분히 가까우면 정지
                    if (target_x_baselink < -position_tolerance)
                    {
                        // 목표를 지나침 (목표가 뒤에 있음)
                        target_reached = true;
                        RCLCPP_INFO(this->get_logger(), "Mission 12: Target passed (behind vehicle)");
                    }
                    else if (target_x_baselink >= -position_tolerance && target_x_baselink <= position_tolerance &&
                             std::fabs(target_y_baselink) <= position_tolerance)
                    {
                        // 목표가 tolerance 범위 내에 있음
                        target_reached = true;
                        RCLCPP_INFO(this->get_logger(), "Mission 12: Target reached (within tolerance)");
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Mission 12: Could not transform target to base_link");
                }

                if (!target_reached)
                {
                    // 아직 목표에 도달하지 않음 - 계속 전진
                    publishSpeedReal(160); // 200mm/s

                    // 진행 상황 모니터링
                    RCLCPP_INFO_THROTTLE(
                        this->get_logger(), *this->get_clock(), 500,
                        "Mission 12: Target(map): (%.3f, %.3f) | Current(map): (%.3f, %.3f) | Dist: %.3fm | "
                        "Target(base_link): (%.3f, %.3f)",
                        target_x, target_y, slam_pose_.x, slam_pose_.y, distance_to_target, target_x_baselink,
                        target_y_baselink);
                }
                else
                {
                    // 목표 위치 도달 - 정지
                    publishSteerInput(0);
                    publishSpeedReal(0);
                    publishControlMode(STOP);

                    RCLCPP_INFO(this->get_logger(), "Mission 12 complete!");
                    RCLCPP_INFO(this->get_logger(), "Target(map): (%.3f, %.3f) | Current(map): (%.3f, %.3f)", target_x,
                                target_y, slam_pose_.x, slam_pose_.y);
                    RCLCPP_INFO(this->get_logger(), "Target(base_link): x=%.3fm, y=%.3fm", target_x_baselink,
                                target_y_baselink);
                    RCLCPP_INFO(this->get_logger(), "Map distance: %.3fm", distance_to_target);

                    mission_position_sent_[12] = false; // Reset for next time
                    transitionToNextMission(15);        // Mission 15로 전환
                }
                break;
            }

            case 15:
            {
                // 정지선까지 비전 레인 제어로 주행
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 15: Lane following with vision control until stop line");

                publishLaneControl(true);
                publishControlMode(LANE_CONTROL); // Vision-based lane control

                // 시작 위치 저장 및 정지선 데이터 초기화 (한 번만)
                if (!mission_position_sent_[15])
                {
                    mission_start_position_mm_[15] = encoder_position_mm_;
                    mission_position_sent_[15] = true;
                    stop_line_position_ = 0.0; // 이전 정지선 데이터 초기화
                    RCLCPP_INFO(this->get_logger(), "Mission 15 started at position: %.1fmm, stop line data reset",
                                encoder_position_mm_);
                }

                // 현재까지 이동한 거리
                double traveled_distance = encoder_position_mm_ - mission_start_position_mm_[15];

                // Vision offset 조절: 1500mm까지는 -80, 그 이후는 0
                if (traveled_distance < 1500.0)
                {
                    publishVisionXteOffset(-97.0); // 왼쪽으로 치우치게
                }
                else
                {
                    publishVisionXteOffset(0.0); // 중앙으로
                }

                // 속도 제어: 2500mm 이후에는 장애물 거리에 따라 속도 조절
                if (traveled_distance < 2500.0)
                {
                    publishSpeedReal(160); // 기본 주행 속도
                }
                else
                {
                    // 2500mm 이후: 장애물이 0.3m 이내에 있으면 정지, 아니면 속도 유지
                    if (obstacle_distance_ < 0.3 && obstacle_distance_ > 0.0)
                    {
                        publishSpeedReal(0);      // 정지
                        publishControlMode(STOP); // 완전 정지 모드
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                             "Mission 15: Obstacle detected at %.2fm - STOPPED", obstacle_distance_);
                    }
                    else
                    {
                        publishSpeedReal(300);            // 속도 유지
                        publishLaneControl(true);         // 레인 제어 재활성화
                        publishControlMode(LANE_CONTROL); // 비전 기반 레인 제어 재활성화
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                             "Mission 15: No obstacle (%.2fm) - Resuming lane control at 300mm/s",
                                             obstacle_distance_);
                    }
                }

                // 최소 2000mm (2m) 주행 전에는 정지선 검출 무시
                if (traveled_distance < 3000.0)
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 15: Traveled %.1f/2000.0 mm, ignoring stop line", traveled_distance);
                }
                else
                {
                    // 2m 이상 주행 후 정지선 검출 시 바로 정지
                    if (stop_line_position_ > 40.0)
                    {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Mission 15: Stop line detected (y=%.1f px, traveled=%.1fmm) - Stopping immediately!",
                            stop_line_position_, traveled_distance);
                        publishLaneControl(false);
                        publishControlMode(STOP);
                        publishSpeedReal(0);

                        mission_position_sent_[15] = false; // Reset for next time
                        transitionToNextMission(16);        // 미션 16번으로 전환
                    }
                    else
                    {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                             "Mission 15: Traveled %.1fmm, waiting for stop line", traveled_distance);
                    }
                }
                break;
            }

            case 16:
            {
                // 미션 16: LIDAR pose 각도를 사용하여 -90도(270도)로 회전
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 16: Rotating to -90 degrees using LIDAR control");

                publishLaneControl(false);
                publishControlMode(LIDAR_CONTROL); // LIDAR-based angle control mode
                publishSpeedReal(100);             // 속도 100mm/s

                // 목표 각도 설정 (한 번만)
                if (!mission_position_sent_[16])
                {
                    publishTargetSlamYaw(270.0); // -90도 = 270도
                    mission_position_sent_[16] = true;
                    RCLCPP_INFO(this->get_logger(), "Mission 16 started: Target angle = 270.0 degrees (-90 degrees)");
                }

                // 현재 SLAM pose yaw 각도 (라디안 -> 도로 변환, 0-360 범위)
                double current_yaw = slam_pose_.yaw_rad * 180.0 / M_PI;
                if (current_yaw < 0.0)
                    current_yaw += 360.0;
                double target_yaw = 270.0;

                // 각도 오차 계산 (-180 ~ 180 범위로 정규화)
                double angle_error = target_yaw - current_yaw;
                if (angle_error > 180.0)
                {
                    angle_error -= 360.0;
                }
                else if (angle_error < -180.0)
                {
                    angle_error += 360.0;
                }

                // 목표 각도 근처 (±3도 이내)에 도달하면 정지
                if (std::fabs(angle_error) <= 3.0)
                {
                    publishSpeedReal(0);
                    publishControlMode(STEER_CONTROL); // 스티어링 직접 제어 모드로 전환
                    publishSteerInput(0);              // 스티어링 각도 0으로 설정

                    // 엔코더 리셋
                    system("ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger &");
                    RCLCPP_INFO(this->get_logger(), "Mission 16: Encoder reset requested, steering set to 0");

                    RCLCPP_INFO(this->get_logger(),
                                "Mission 16 complete: Reached target angle (current=%.1f°, target=%.1f°, error=%.1f°)",
                                current_yaw, target_yaw, angle_error);

                    mission_position_sent_[16] = false; // Reset for next time
                    transitionToNextMission(17);        // 미션 17로 전환 (SLAM 중지)
                }
                else
                {
                    // 진행 상황 모니터링
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 16: Current angle=%.1f°, Target=%.1f°, Error=%.1f°", current_yaw,
                                         target_yaw, angle_error);
                }
                break;
            }

            case 17:
            {
                // 미션 17: SLAM Toolbox 중지 및 엔코더 리셋
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 17: Stopping SLAM Toolbox and resetting encoder");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL); // 스티어링 직접 제어 모드로 전환
                publishSteerInput(0);              // 스티어링 각도 0으로 설정
                publishSpeedReal(0);

                // SLAM Toolbox 중지 및 엔코더 리셋 (한 번만)
                static bool mission17_started = false;
                static rclcpp::Time mission17_start_time;
                static bool encoder_reset_sent = false;

                if (!mission17_started)
                {
                    // SLAM Toolbox 노드를 완전히 종료 (kill)
                    system("pkill -9 -f 'async_slam_toolbox_node' &");
                    RCLCPP_INFO(this->get_logger(), "Mission 17: SLAM Toolbox node killed (will restart in Mission 18)");

                    mission17_start_time = this->now();
                    mission17_started = true;
                    encoder_reset_sent = false;
                }

                double elapsed_time = (this->now() - mission17_start_time).seconds();

                // 0.2초마다 엔코더 값 확인
                if (elapsed_time >= 0.2)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 17: Encoder value after reset = %.2f mm (elapsed: %.2fs)",
                                encoder_position_mm_, elapsed_time);

                    // 엔코더 값이 0이 아니면 리셋 (절대값 50mm 이상)
                    if (std::abs(encoder_position_mm_) > 50.0)
                    {
                        system("ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger &");
                        RCLCPP_WARN(this->get_logger(),
                                    "Mission 17: Encoder NOT zero (%.2fmm) - Requesting reset again!",
                                    encoder_position_mm_);
                        mission17_start_time = this->now(); // 타이머 리셋
                        elapsed_time = 0.0;
                    }
                    else
                    {
                        // 엔코더 값이 0에 가까우면 Mission 100로 전환
                        RCLCPP_INFO(this->get_logger(),
                                    "========================================");
                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 17: ✓ Encoder reset CONFIRMED!");
                        RCLCPP_INFO(this->get_logger(),
                                    "  Final encoder value: %.2f mm", encoder_position_mm_);
                        RCLCPP_INFO(this->get_logger(),
                                    "  Reset verified in %.2f seconds", elapsed_time);
                        RCLCPP_INFO(this->get_logger(),
                                    "========================================");
                        mission17_started = false;
                        encoder_reset_sent = false;
                        transitionToNextMission(18);
                    }
                }
                else
                {
                    // 첫 리셋 요청
                    if (!encoder_reset_sent)
                    {
                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 17: Encoder before reset = %.2f mm", encoder_position_mm_);
                        system("ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger &");
                        encoder_reset_sent = true;
                        RCLCPP_INFO(this->get_logger(), "Mission 17: Initial encoder reset requested");
                    }
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                         "Mission 17: Waiting for encoder update... %.2fs (current=%.2fmm)",
                                         elapsed_time, encoder_position_mm_);
                }
                break;
            }

            case 18:
            {
                // 미션 18: SLAM Toolbox 재시작 후 미션 19로 전환
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 18: Restarting SLAM Toolbox");

                publishLaneControl(false);
                publishControlMode(STOP);
                publishSpeedReal(0);

                // SLAM Toolbox 시작 (한 번만)
                static bool mission18_slam_started = false;
                static rclcpp::Time mission18_start_time;

                if (!mission18_slam_started)
                {
                    // SLAM Toolbox 노드를 다시 launch (백그라운드에서 실행)
                    system("ros2 launch slam_toolbox_config online_async_launch.py > /dev/null 2>&1 &");
                    RCLCPP_INFO(this->get_logger(), "Mission 18: SLAM Toolbox node relaunched");

                    mission18_start_time = this->now();
                    mission18_slam_started = true;
                    slam_pose_received_ = false; // Reset flag to wait for new pose
                }

                double elapsed = (this->now() - mission18_start_time).seconds();

                // SLAM이 시작되었는지 확인 (3초 대기 후 확인 - 노드 launch 시간 필요)
                if (elapsed >= 3.0)
                {
                    if (slam_pose_received_)
                    {
                        RCLCPP_INFO(this->get_logger(),
                                    "========================================");
                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 18: ✓ SLAM Toolbox running confirmed!");
                        RCLCPP_INFO(this->get_logger(),
                                    "  SLAM pose: x=%.2f m, y=%.2f m, yaw=%.1f°",
                                    slam_pose_.x, slam_pose_.y, RAD2DEG(slam_pose_.yaw_rad));
                        RCLCPP_INFO(this->get_logger(),
                                    "  Note: Mission 19 will use this as starting reference (0,0,0)");
                        RCLCPP_INFO(this->get_logger(),
                                    "========================================");
                        mission18_slam_started = false;
                        transitionToNextMission(19);
                    }
                    else
                    {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                            "Mission 18: Waiting for SLAM pose... %.2fs (no pose received yet)",
                                            elapsed);
                    }
                }
                else
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 18: Waiting for SLAM node to launch... %.2fs / 3.0s",
                                         elapsed);
                }
                break;
            }

            case 19:
            {
                // 미션 19: SLAM 좌표 기준 X 방향 20cm 이동 후 미션 20으로 전환
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 19: Moving 20cm forward (SLAM X coordinate)");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);
                publishSteerInput(0);

                // 미션 시작 시 SLAM X 위치 저장
                if (!mission_position_sent_[19])
                {
                    mission_start_slam_x_[19] = slam_pose_.x;
                    mission_position_sent_[19] = true;
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 19: Starting SLAM X = %.3f m", slam_pose_.x);
                }

                // SLAM X 방향 이동량 계산 (절댓값 사용)
                double dx = std::abs(slam_pose_.x - mission_start_slam_x_[19]);
                double dx_cm = dx * 100.0;

                // 목표: X 방향으로 10cm 이동
                if (dx < 0.10)
                {
                    publishSpeedReal(150);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 19: X moved %.1f cm / 10.0 cm", dx_cm);
                }
                else
                {
                    publishSpeedReal(0);
                    publishSteerInput(0);
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 19: ✓ Completed! X moved %.1f cm", dx_cm);
                    mission_position_sent_[19] = false;
                    transitionToNextMission(20);
                }
                break;
            }

            ////////////////////////////////////////////////// 추월구간 /////////////////////////////////////////////
            case 20:
            {
                // 미션 20: 왼쪽 벽 감지하여 각도 계산 (SLAM은 미션 18에서 시작됨)
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 20: Detecting left wall angle");

                publishLaneControl(false);
                publishControlMode(STOP);
                publishSpeedReal(0);

                // SLAM pose가 수신되었는지 확인
                if (slam_pose_received_)
                {
                    // 왼쪽 벽 감지 및 각도 계산
                    double left_wall_angle = detectLeftWallAngle();

                    if (left_wall_angle != -999.0) // 유효한 각도 감지됨
                    {
                        // 벽과 평행하도록 목표 각도 설정
                        // 왼쪽 벽이 90도(차량 왼쪽)에 있어야 평행
                        double angle_to_parallel = left_wall_angle - 90.0;

                        // 현재 SLAM yaw에 보정 각도를 더함
                        double current_yaw_deg = slam_pose_.yaw_rad * 180.0 / M_PI;
                        if (current_yaw_deg < 0.0)
                            current_yaw_deg += 360.0;
                        double target_yaw = current_yaw_deg + angle_to_parallel;

                        // 0-360 범위로 정규화
                        if (target_yaw < 0.0)
                            target_yaw += 360.0;
                        if (target_yaw >= 360.0)
                            target_yaw -= 360.0;

                        // 보정값 저장
                        wall_angle_correction_ = angle_to_parallel;

                        // 벽 각도 상세 정보 출력
                        RCLCPP_INFO(this->get_logger(),
                                    "========================================");
                        RCLCPP_INFO(this->get_logger(),
                                    "Mission 20: ✓ 왼쪽 벽 검출 완료!");
                        RCLCPP_INFO(this->get_logger(),
                                    "  검출된 벽 각도 (차량 기준): %.2f°", left_wall_angle);
                        RCLCPP_INFO(this->get_logger(),
                                    "  현재 차량 yaw: %.1f°", current_yaw_deg);
                        RCLCPP_INFO(this->get_logger(),
                                    "  평행까지 필요한 회전: %.1f°", angle_to_parallel);
                        RCLCPP_INFO(this->get_logger(),
                                    "  목표 yaw (벽과 평행): %.1f°", target_yaw);
                        RCLCPP_INFO(this->get_logger(),
                                    "========================================");

                        // 미션 20 완료 - 미션 100으로 전환
                        mission_position_sent_[20] = false;
                        transitionToNextMission(21);
                    }
                    else
                    {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                             "Mission 20: Waiting for left wall detection...");
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Mission 20: Waiting for SLAM pose... (SLAM may not be running!)");
                }

                break;
            }

            case 21:
            {
                // 조향각 20도로 500mm 주행 후 정지
                executeSteerDistanceMission(21, 23, 500.0, 150, 22);
                break;
            }

            case 22:
            {
                // 미션 22: heading angle이 -25도가 될 때까지 전진
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 22: Moving forward until heading = -25°");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);

                // SLAM yaw를 도 단위로 변환
                double current_yaw_deg = slam_pose_.yaw_rad * 180.0 / M_PI;

                // -180~180 범위로 정규화
                while (current_yaw_deg > 180.0) current_yaw_deg -= 360.0;
                while (current_yaw_deg < -180.0) current_yaw_deg += 360.0;

                // 목표: yaw가 -25도 (±3도 허용)
                if (current_yaw_deg <= -22.0)
                {
                    // -25도 도달 - Mission 23으로 전환
                    publishSteerInput(0);
                    publishSpeedReal(0);
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 22: ✓ Heading reached -25° (current: %.1f°)", current_yaw_deg);
                    transitionToNextMission(23);
                }
                else
                {
                    // 아직 -25도 아님 - 조향각 0도로 직진
                    publishSteerInput(0);
                    publishSpeedReal(150);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 22: Current heading = %.1f° (target: -25°)", current_yaw_deg);
                }
                break;
            }

            case 23:
            {
                // 라이다 장애물 50cm 앞에서 정지 (왼쪽으로 치우치게 offset -50)
                RCLCPP_INFO_ONCE(this->get_logger(),
                                 "Mission 23: Lane following until obstacle at 50cm with left offset");

                publishLaneControl(true);
                publishControlMode(LANE_CONTROL); // Vision-based lane control
                publishVisionXteOffset(-40.0);    // 왼쪽으로 40px 치우치게

                // 장애물까지 거리가 50cm(0.5m)보다 크면 주행
                if (obstacle_distance_ > 0.7)
                {
                    publishSpeedReal(200); // 주행 속도 설정
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 23: Obstacle distance %.2fm, continuing", obstacle_distance_);
                }
                else
                {
                    // 장애물이 50cm 이내 - 정지하고 Mission 24로 전환
                    publishSpeedReal(0);
                    publishLaneControl(false);
                    publishControlMode(STOP);
                    publishVisionXteOffset(0.0); // Offset을 0으로 복귀

                    RCLCPP_INFO(this->get_logger(), "Mission 23 complete: Stopped at obstacle distance %.2fm",
                                obstacle_distance_);
                    transitionToNextMission(24);
                }
                break;
            }

            case 24:
            {
                // 미션 24: 조향각 -30도로 유지하다가 heading 30도 되면 Mission 25로 전환
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 24: Steering -30° until heading = 30°");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);

                // SLAM yaw를 도 단위로 변환
                double current_yaw_deg = slam_pose_.yaw_rad * 180.0 / M_PI;

                // -180~180 범위로 정규화
                while (current_yaw_deg > 180.0) current_yaw_deg -= 360.0;
                while (current_yaw_deg < -180.0) current_yaw_deg += 360.0;

                // 목표: yaw가 30도 (±3도 허용)
                if (std::abs(current_yaw_deg - 30.0) <= 3.0)
                {
                    // 30도 도달 - 조향각 0도로 변경하고 Mission 25로 전환
                    publishSteerInput(0);
                    publishSpeedReal(0);
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 24: ✓ Heading reached 30° (current: %.1f°)", current_yaw_deg);
                    transitionToNextMission(25);
                }
                else
                {
                    // 아직 30도 아님 - 조향각 -30도로 계속 주행
                    publishSteerInput(-30);
                    publishSpeedReal(150);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 24: Current heading = %.1f° (target: 30°)", current_yaw_deg);
                }
                break;
            }

            case 25:
            {
                // 미션 25: 조향각 0도로 직진하면서 SLAM Y 좌표가 0 근처까지 이동 후 Mission 100으로 전환
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 25: Moving straight until SLAM Y = 0");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);
                publishSteerInput(0); // 조향각 0도 ()

                // SLAM Y 좌표 확인
                double current_y = slam_pose_.y;
                double current_y_cm = current_y * 100.0;

                // 목표: Y가 0 근처 (±5cm 허용) 10으로 수정함
                if (std::abs(current_y) <= 0.10)
                {
                    // Y = 0 도달 - 정지하고 Mission 26으로 전환
                    publishSpeedReal(0);
                    publishSteerInput(0);
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 25: ✓ SLAM Y reached 0 (current: %.1f cm)", current_y_cm);
                    transitionToNextMission(26);
                }
                else
                {
                    // 아직 Y = 0 아님 - 계속 직진
                    publishSpeedReal(150);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 25: SLAM Y = %.1f cm (target: 0 cm)", current_y_cm);
                }
                break;
            }

            case 26:
            {
                // 미션 26: 조향각 -35도로 유지하다가 heading 0도 되면 Mission 27로 전환
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 26: Steering -35° until heading = 0°");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);
                publishSteerInput(30); // 조향각 30 우회전

                // SLAM yaw를 도 단위로 변환
                double current_yaw_deg = slam_pose_.yaw_rad * 180.0 / M_PI;

                // -180~180 범위로 정규화
                while (current_yaw_deg > 180.0) current_yaw_deg -= 360.0;
                while (current_yaw_deg < -180.0) current_yaw_deg += 360.0;

                // 목표: yaw가 0도 (±3도 허용)
                if (std::abs(current_yaw_deg) <= 3.0)
                {
                    // 0도 도달 - 조향각 0도로 변경하고 Mission 27로 전환
                    publishSteerInput(0);
                    publishSpeedReal(0);
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 26: ✓ Heading reached 0° (current: %.1f°)", current_yaw_deg);
                    transitionToNextMission(27);
                }
                else
                {
                    // 아직 0도 아님 - 조향각 -35도로 계속 주행
                    publishSteerInput(-35);
                    publishSpeedReal(150);
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                         "Mission 26: Current heading = %.1f° (target: 0°)", current_yaw_deg);
                }
                break;
            }

            case 27:
            {
                // 미션 27: LIDAR CONTROL 모드로 heading angle 0도 제어하며 직진
                // 라인이 하나라도 검출되면 Mission 100으로 이동
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 27: LIDAR_CONTROL mode - heading to 0°, stop on line detection");

                publishLaneControl(false);
                publishControlMode(LIDAR_CONTROL); // LIDAR-based yaw control mode
                publishSpeedReal(150); // 속도 150mm/s

                // 목표 각도 설정 (한 번만)
                static bool mission27_target_set = false;
                if (!mission27_target_set)
                {
                    publishTargetSlamYaw(0.0); // 목표 0도
                    mission27_target_set = true;
                    RCLCPP_INFO(this->get_logger(), "Mission 27: Target yaw set to 0°");
                }

                // 현재 SLAM yaw 각도
                double current_yaw_deg = slam_pose_.yaw_rad * 180.0 / M_PI;

                // 라인 검출 확인 (왼쪽 또는 오른쪽 중 하나라도 검출되면)
                if (lane_status_left_ || lane_status_right_)
                {
                    // 라인 검출 - 정지하고 Mission 100으로 전환
                    publishSpeedReal(0);
                    publishSteerInput(0);
                    publishControlMode(STOP);

                    RCLCPP_INFO(this->get_logger(),
                                "Mission 27: Line detected! (Left=%s, Right=%s) - Moving to Mission 100",
                                lane_status_left_ ? "DETECTED" : "CLEAR",
                                lane_status_right_ ? "DETECTED" : "CLEAR");

                    mission27_target_set = false; // Reset for next time
                    transitionToNextMission(100);
                }
                else
                {
                    // 라인 미검출 - 계속 직진
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                         "Mission 27: Current heading = %.1f° (target: 0°), No line detected",
                                         current_yaw_deg);
                }

                break;
            }

            ////////////////////////////////////////////////// 라이다 미로 구간
            ////////////////////////////////////////////////
            case 30:

                break;

            ////////////////////////////////////////////////// LiDAR Cone Control 구간
            ////////////////////////////////////////////////
            case 31:
            {
                // 미션 31: LiDAR Cone Control - 콘 추종 주행
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 31: CONE_CONTROL mode - following cones");

                publishLaneControl(false);
                publishControlMode(CONE_CONTROL); // Cone following control mode
                publishSpeedReal(200); // 속도 200mm/s

                // XTE 모니터링 (1초마다 로그)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Mission 31: Cone XTE = %.3f m", lidar_cone_xte_);

                // 종료 조건: 수동 전환 또는 특정 조건 추가 가능
                // 예: 콘이 더 이상 감지되지 않거나, 일정 거리 주행 후
                // 현재는 계속 주행 (수동 미션 전환 필요)

                break;
            }

            ////////////////////////////////////////////////// 주차 구간 /////////////////////////////////////////////
            case 40:

                break;

            ////////////////////////////////////////////////// 벽 미로 구간
            ////////////////////////////////////////////////
            case 50:

                break;

            ////////////////////////////////// 정지 명령 /////////////////////////////////
            case 100:
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 100: Vehicle stopped");
                publishSteerInput(35); // 조향각 35도
                publishSpeedReal(0);
                break;

            default:
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Unknown mission flag: %d",
                                     mission_flag_);
                publishControlMode(STOP);
                publishSpeedReal(0);
                break;
            }
            // Publish current mission status for GUI
            publishMissionStatus();
        }
        else
        {
            // Stop all control
            publishLaneControl(false);
            publishSpeedReal(0);

            // Publish stopped status
            publishMissionStatus();
        }
    }

    void publishMissionStatus()
    {
        std_msgs::msg::Int16 msg;
        msg.data = mission_flag_;
        mission_status_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Mission flag: %d", mission_flag_);
    }

    // Helper function for STEER_CONTROL + encoder distance missions
    void executeSteerDistanceMission(int mission_num, int steer_angle, double target_distance_mm, int speed_mm_s,
                                     int next_mission)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Mission %d: Moving %.1fmm with steering angle %d degrees at %d mm/s",
                         mission_num, target_distance_mm, steer_angle, speed_mm_s);

        publishLaneControl(false);
        publishControlMode(STEER_CONTROL);
        publishSteerInput(steer_angle);

        // Save start position (once only)
        if (!mission_position_sent_[mission_num])
        {
            mission_start_position_mm_[mission_num] = encoder_position_mm_;
            mission_position_sent_[mission_num] = true;
            RCLCPP_INFO(this->get_logger(), "Mission %d started at position: %.1fmm", mission_num,
                        encoder_position_mm_);
        }

        // Calculate traveled distance
        double traveled_distance = encoder_position_mm_ - mission_start_position_mm_[mission_num];

        // Check if target reached
        if (traveled_distance < target_distance_mm)
        {
            publishSpeedReal(speed_mm_s);
        }
        else
        {
            // Target reached - stop and transition
            publishSpeedReal(0);
            publishControlMode(STOP);
            RCLCPP_INFO(this->get_logger(), "Mission %d completed: traveled %.1fmm", mission_num, traveled_distance);
            transitionToNextMission(next_mission);
        }
    }

    // Transform point from map frame to base_link frame
    bool transformPointToBaseLink(double map_x, double map_y, double& baselink_x, double& baselink_y)
    {
        try
        {
            // Get transform from map to base_link
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero);

            // Create target point in map frame
            geometry_msgs::msg::PointStamped point_map;
            point_map.header.frame_id = "map";
            point_map.header.stamp = this->now();
            point_map.point.x = map_x;
            point_map.point.y = map_y;
            point_map.point.z = 0.0;

            // Transform to base_link frame
            geometry_msgs::msg::PointStamped point_baselink;
            tf2::doTransform(point_map, point_baselink, transform_stamped);

            baselink_x = point_baselink.point.x;
            baselink_y = point_baselink.point.y;

            return true;
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_DEBUG(this->get_logger(), "Could not transform point to base_link: %s", ex.what());
            return false;
        }
    }

    // Check if current mission has reached the end mission flag
    bool shouldStopAtCurrentMission()
    {
        return (mission_flag_ >= end_mission_flag_);
    }

    // Transition to next mission or stop if end is reached
    void transitionToNextMission(int next_mission)
    {
        if (mission_flag_ >= end_mission_flag_)
        {
            // Stop the race if we've completed the end mission
            RCLCPP_INFO(this->get_logger(), "Reached end mission (%d) - Stopping race", end_mission_flag_);
            run_flag_ = false;
            publishSpeedReal(0);
            publishControlMode(STOP);
        }
        else
        {
            mission_flag_ = next_mission;
            RCLCPP_INFO(this->get_logger(), "Transitioning to Mission %d", next_mission);
        }
    }

    void publishControlMode(int mode)
    {
        std_msgs::msg::Int8 msg;
        msg.data = mode;
        yaw_control_mode_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Control mode: %d", mode);
    }

    void publishSpeedReal(int speed)
    {
        amap_powerpack_single_driver::msg::TargetSpeed msg;
        msg.target_speed = static_cast<float>(speed);
        car_control_speed_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Real speed command: %d mm/s", speed);
    }

    void publishSteeringAngle(float angle_degree)
    {
        amap_powerpack_single_driver::msg::SteeringAngle msg;
        msg.steering_angle = angle_degree;
        car_control_steering_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Steering angle command: %.1f degrees", angle_degree);
    }

    void publishSteerInput(int steer_angle)
    {
        std_msgs::msg::Int16 msg;
        msg.data = steer_angle;
        steer_input_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Steer input for STEER_CONTROL mode: %d degrees", steer_angle);
    }

    void publishVisionXteOffset(float offset)
    {
        std_msgs::msg::Float32 msg;
        msg.data = offset;
        vision_xte_offset_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Vision XTE offset: %.1f px", offset);
    }

    void publishLaneControl(bool enable)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = enable;
        lane_control_flag_pub_->publish(msg);
    }

    void publishTargetAngle(float angle)
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = angle;
        target_angle_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Target angle: %.1f degrees", angle);
    }

    void publishTargetAngularVelocity(float angular_velocity)
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = angular_velocity;
        target_angular_velocity_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Target angular velocity: %.3f rad/s", angular_velocity);
    }

    void publishTargetSlamYaw(float yaw_degrees)
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = yaw_degrees;
        target_slam_yaw_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Target SLAM yaw: %.1f degrees", yaw_degrees);
    }

    void publishMissionTargetMarker(double x, double y, int mission_number, const std::string& color = "green")
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "mission_target";
        marker.id = mission_number;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.1;  // Slightly above ground

        // Orientation (not used for sphere, but required)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Scale (20cm diameter sphere)
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        // Color
        if (color == "red") {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (color == "blue") {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        } else if (color == "yellow") {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        } else {  // default green
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        marker.color.a = 0.8;  // Semi-transparent

        // Lifetime (0 = forever, or set duration)
        marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persist until deleted

        mission_target_marker_pub_->publish(marker);
        RCLCPP_DEBUG(this->get_logger(), "Published target marker for Mission %d at (%.2f, %.2f)",
                     mission_number, x, y);
    }

    void deleteMissionTargetMarker(int mission_number)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "mission_target";
        marker.id = mission_number;
        marker.action = visualization_msgs::msg::Marker::DELETE;

        mission_target_marker_pub_->publish(marker);
        RCLCPP_DEBUG(this->get_logger(), "Deleted target marker for Mission %d", mission_number);
    }

    void publishObstacleEnable(bool enable)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = enable;
        obstacle_enable_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Obstacle detection: %s", enable ? "ENABLED" : "DISABLED");
    }

    double detectLeftWallAngle()
    {
        // map_wall_detector 방식을 사용하여 왼쪽 벽 감지
        // LiDAR 데이터에서 왼쪽 영역(60-120도) 점들을 추출하여 Hough 변환 수행

        if (lidar_ranges_.empty())
        {
            return -999.0; // 유효하지 않은 값
        }

        // 왼쪽 영역 점들 추출 (60-120도)
        std::vector<std::pair<double, double>> left_points; // (x, y) 좌표

        for (size_t i = 0; i < lidar_ranges_.size(); i++)
        {
            double range = lidar_ranges_[i];
            if (range > 0.1 && range < 3.0) // 0.1m ~ 3m 유효 범위
            {
                // 올바른 각도 계산: angle_min + i * angle_increment
                double angle_rad = lidar_angle_min_ + (i * lidar_angle_increment_);

                // 왼쪽 영역만 선택 (60-120도 = 1.047-2.094 rad)
                double angle_deg = angle_rad * 180.0 / M_PI;

                // 360도 범위로 정규화
                while (angle_deg < 0.0)
                    angle_deg += 360.0;
                while (angle_deg >= 360.0)
                    angle_deg -= 360.0;

                if (angle_deg >= 60.0 && angle_deg <= 120.0)
                {
                    double x = range * cos(angle_rad);
                    double y = range * sin(angle_rad);
                    left_points.push_back({x, y});
                }
            }
        }

        if (left_points.size() < 10) // 최소 10개 점 필요
        {
            return -999.0;
        }

        // 간단한 Hough 변환 수행
        const int theta_bins = 180;
        const int rho_bins = 400;
        const double max_rho = 10.0;

        std::vector<std::vector<int>> accumulator(rho_bins, std::vector<int>(theta_bins, 0));

        // Hough 변환 수행
        for (const auto& point : left_points)
        {
            for (int t = 0; t < theta_bins; t++)
            {
                double theta = t * M_PI / 180.0;
                double rho = point.first * cos(theta) + point.second * sin(theta);

                int rho_idx = (rho + max_rho) * rho_bins / (2 * max_rho);

                if (rho_idx >= 0 && rho_idx < rho_bins)
                {
                    accumulator[rho_idx][t]++;
                }
            }
        }

        // 상위 직선들 찾기 (가장 긴 직선들)
        struct Line
        {
            int votes;
            int theta_idx;
            int rho_idx;
            double theta_deg;
            double rho;
        };
        std::vector<Line> detected_lines;

        for (int r = 0; r < rho_bins; r++)
        {
            for (int t = 0; t < theta_bins; t++)
            {
                if (accumulator[r][t] >= 5) // 최소 투표수
                {
                    Line line;
                    line.votes = accumulator[r][t];
                    line.theta_idx = t;
                    line.rho_idx = r;
                    line.theta_deg = t; // 각도 (0-180)
                    line.rho = (r * 2 * max_rho / rho_bins) - max_rho;
                    detected_lines.push_back(line);
                }
            }
        }

        if (detected_lines.empty())
        {
            RCLCPP_WARN(this->get_logger(), "검출된 직선이 없습니다.");
            return -999.0;
        }

        // 투표수(직선 길이) 기준으로 정렬
        std::sort(detected_lines.begin(), detected_lines.end(),
                  [](const Line& a, const Line& b) { return a.votes > b.votes; });

        // 검출된 직선들 출력
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "라이다 직선 검출 결과 (왼쪽 영역 60-120°):");

        int num_to_show = std::min(5, (int)detected_lines.size()); // 상위 5개만 표시
        for (int i = 0; i < num_to_show; i++)
        {
            const auto& line = detected_lines[i];

            // Hough theta는 벽의 법선 방향 (0-180°)
            double hough_theta = line.theta_deg;

            // 벽의 실제 방향 = 법선 방향 - 90° (법선에 수직)
            double wall_direction = hough_theta - 90.0;

            // 음수면 양수로 변환
            if (wall_direction < 0.0)
                wall_direction += 180.0;

            RCLCPP_INFO(this->get_logger(),
                        "  직선 %d: 길이(votes)=%d, Hough theta(법선)=%.1f°, 벽 방향(차량기준)=%.1f°, rho=%.2fm",
                        i + 1, line.votes, hough_theta, wall_direction, line.rho);
        }
        RCLCPP_INFO(this->get_logger(), "========================================");

        // 가장 긴 직선(최대 투표수) 사용
        const auto& best_line = detected_lines[0];
        double hough_theta_deg = best_line.theta_deg;
        double wall_direction_deg = hough_theta_deg - 90.0;

        if (wall_direction_deg < 0.0)
            wall_direction_deg += 180.0;

        RCLCPP_INFO(this->get_logger(),
                    ">>> 가장 긴 직선: 벽 방향(차량 전방 기준) = %.1f°",
                    wall_direction_deg);

        return wall_direction_deg;
    }

    void slamToolboxStart()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "start";
        slam_command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "SLAM Toolbox: START command sent");
    }

    void slamToolboxStop()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "stop";
        slam_command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "SLAM Toolbox: STOP command sent");
    }

    void slamToolboxReset()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "reset";
        slam_command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "SLAM Toolbox: RESET command sent (map and position cleared)");
    }

    void slamToolboxResetPose()
    {
        // Reset only the pose to (0, 0, 0) without clearing the map
        system("ros2 service call /slam_toolbox/reset std_srvs/srv/Empty &");
        RCLCPP_INFO(this->get_logger(), "SLAM Toolbox: Pose reset to (0,0,0) requested");
    }

    // ==================== Parameters ====================

    // ==================== Sensor Data ====================
    double imu_heading_angle_degree_;      // Current IMU heading angle in degrees (0-360)
    double lidar_front_obstacle_distance_; // Front obstacle distance from lidar (meters)
    std::vector<float> lidar_ranges_;      // All lidar ranges for wall detection
    double lidar_angle_min_;               // Lidar angle_min from LaserScan
    double lidar_angle_increment_;         // Lidar angle_increment from LaserScan
    double stop_line_position_;            // Stop line position from vision (pixels)
    double wall_angle_correction_;         // Wall angle correction from Mission 20 (degrees)
    bool lane_status_left_;                // Left lane obstacle status (false = CLEAR, true = BLOCKED)
    bool lane_status_center_;              // Center lane obstacle status (false = CLEAR, true = BLOCKED)
    bool lane_status_right_;               // Right lane obstacle status (false = CLEAR, true = BLOCKED)

    // SLAM Toolbox pose data (map frame)
    SlamPose slam_pose_;                 // SLAM pose in map frame (x, y, yaw_rad)
    rclcpp::Time slam_pose_last_update_; // 마지막 SLAM pose 업데이트 시간
    bool slam_pose_received_;            // SLAM pose가 한 번이라도 수신되었는지 여부

    // Obstacle detection data
    double obstacle_distance_; // Front obstacle distance (meters)
    bool obstacle_detected_;   // Obstacle detected flag

    // Lidar cone control data
    double lidar_cone_xte_;    // Cross-track error from lidar cone detection (meters)

    // IMU calibration data
    std::vector<double> imu_calibration_samples_; // IMU samples for calibration
    double imu_angle_offset_;                     // IMU angle offset for calibration
    bool imu_calibrated_;                         // IMU calibration complete flag
    bool imu_offset_reset_sent_;                  // Flag to track if offset reset was sent

    // Encoder data (from powerpack driver)
    double encoder_position_mm_; // Vehicle position in millimeters (accumulated)
    double encoder_speed_mms_;   // Vehicle speed in mm/s

    // Mission control data - using arrays for scalability
    std::array<double, 100> mission_start_position_mm_; // Starting positions for all missions
    std::array<double, 100> mission_start_slam_x_;      // Starting SLAM X positions for all missions
    std::array<double, 100> mission_start_slam_y_;      // Starting SLAM Y positions for all missions
    std::array<bool, 100> mission_position_sent_;       // Flags to track if position command was sent
    bool encoder_reset_requested_;                      // Flag to track if encoder reset was requested

    // Mission 4, 5, 6 specific control data
    double mission4_saved_angle_;      // Saved IMU angle at mission 4 (reference angle)
    bool mission4_angle_saved_;        // Flag to track if angle was saved
    int mission4_no_obstacle_count_;   // Counter for consecutive "no obstacle" detections
    int mission4_obstacle_count_;      // Counter for consecutive "obstacle" detections
    rclcpp::Time mission4_start_time_; // Mission 4 start time for minimum wait period
    rclcpp::Time mission6_start_time_; // Mission 6 start time for 0.5s stop
    bool mission6_started_;            // Mission 6 started flag
    bool mission6_encoder_reset_sent_; // Mission 6 encoder reset sent flag

    // ==================== Mission Control ====================
    int mission_flag_;       // Current mission state
    int start_mission_flag_; // Mission to start from (set by GUI)
    int end_mission_flag_;   // Mission to end at (set by GUI)
    bool run_flag_;          // Race start/stop flag

    // ==================== Subscribers - Real Hardware ====================
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_real_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub_;

    // ==================== Subscribers - Common ====================
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_flag_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stop_line_position_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_distance_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lidar_cone_xte_sub_;
    rclcpp::Subscription<amap_powerpack_single_driver::msg::EncoderStatus>::SharedPtr encoder_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr start_mission_flag_sub_; // Start mission config from GUI
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr end_mission_flag_sub_;   // End mission config from GUI

    // ==================== Publishers ====================
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr yaw_control_mode_pub_; // Steering control mode
    rclcpp::Publisher<amap_powerpack_single_driver::msg::TargetSpeed>::SharedPtr
        car_control_speed_pub_; // Speed command (Real hardware)
    rclcpp::Publisher<amap_powerpack_single_driver::msg::SteeringAngle>::SharedPtr
        car_control_steering_pub_;                                               // Steering angle command (direct)
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steer_input_pub_;         // Steer input for STEER_CONTROL mode
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vision_xte_offset_pub_; // Vision XTE offset
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lane_control_flag_pub_;    // Lane control enable/disable
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_angle_pub_;      // Target angle for IMU control
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        target_angular_velocity_pub_;                                       // Target angular velocity for STEER_CONTROL
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_enable_pub_; // Obstacle detection enable/disable
    rclcpp::Publisher<amap_powerpack_single_driver::msg::TargetPositionRelative>::SharedPtr
        target_position_relative_pub_;                                         // Relative position control
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_offset_pub_;      // IMU angle offset
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mission_status_pub_;    // Mission status for GUI
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr slam_command_pub_;     // SLAM command (start/stop/reset)
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_slam_yaw_pub_; // Target SLAM yaw for LIDAR_CONTROL
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mission_target_marker_pub_; // Mission target marker visualization

    // ==================== TF2 ====================
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ==================== Timer ====================
    rclcpp::TimerBase::SharedPtr timer_; // Main control loop timer (10Hz)
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlNode>());
    rclcpp::shutdown();
    return 0;
}