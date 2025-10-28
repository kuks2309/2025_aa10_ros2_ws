#include "amap_powerpack_single_driver/msg/encoder_status.hpp"
#include "amap_powerpack_single_driver/msg/target_position_relative.hpp"
#include "amap_powerpack_single_driver/msg/target_speed.hpp"
#include "amap_powerpack_single_driver/msg/steering_angle.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
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
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <numeric>
#include <vector>

#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x) / 180. * M_PI)

// Yaw Control Modes (must match aa10_car_yaw_control definitions)
#define IMU_CONTROL 0       // IMU-based yaw control
#define LANE_CONTROL 1      // Vision-based lane following (uses /xte/vision from ai_line_detection)
#define MAZE_CONTROL 2      // Maze navigation control
#define STEER_CONTROL 3     // Direct steering angle control

// Additional mission control modes
#define STOP 99             // Stop mode (not used by yaw_control)
#define WALL_FOLLOWING 5
#define OBSTACLE_DETECT 6

class MissionControlNode : public rclcpp::Node
{
  public:
    MissionControlNode() : Node("mission_control_node")
    {
        // Parameters for real hardware
        this->declare_parameter("imu_topic", "/handsfree/imu_yaw_correction_degree");
        this->declare_parameter("lidar_topic", "/scan");
        this->declare_parameter("camera_topic", "/camera/image_raw");

        // Initialize variables
        imu_heading_angle_degree_ = 0.0;
        lidar_front_obstacle_distance_ = 0.0;
        stop_line_position_ = 0.0;
        mission_flag_ = 0;
        start_mission_flag_ = 0; // Default: start from mission 0
        end_mission_flag_ = 7;   // Default: end at mission 7
        run_flag_ = false;
        lane_status_left_ = false;   // false = CLEAR, true = BLOCKED
        lane_status_center_ = false; // false = CLEAR, true = BLOCKED
        lane_status_right_ = false;  // false = CLEAR, true = BLOCKED
        obstacle_distance_ = 0.0;
        obstacle_detected_ = false;
        imu_angle_offset_ = 0.0;
        imu_calibrated_ = false;
        imu_offset_reset_sent_ = false;
        imu_calibration_samples_.clear();
        encoder_position_mm_ = 0.0;
        encoder_speed_mms_ = 0.0;
        mission3_start_position_mm_ = 0.0;
        mission3_position_sent_ = false;
        encoder_reset_requested_ = false;
        mission4_saved_angle_ = 0.0;
        mission4_angle_saved_ = false;
        mission5_start_position_mm_ = 0.0;
        mission5_position_sent_ = false;
        mission6_start_position_mm_ = 0.0;
        mission6_position_sent_ = false;

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

        // Create steering angle publisher
        car_control_steering_pub_ =
            this->create_publisher<amap_powerpack_single_driver::msg::SteeringAngle>("/car_control/steering_angle", 5);

        // Create position control publisher
        target_position_relative_pub_ =
            this->create_publisher<amap_powerpack_single_driver::msg::TargetPositionRelative>(
                "/car_control/target_position_relative", 5);

        // Create IMU offset publisher
        imu_offset_pub_ = this->create_publisher<std_msgs::msg::Float32>("/imu_angle_offset", 5);

        // Create mission status publisher for GUI
        mission_status_pub_ = this->create_publisher<std_msgs::msg::Int16>("/mission_control/mission_flag", 5);

        // Timer for main control loop
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), // 10Hz
                                         std::bind(&MissionControlNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Mission Control Node initialized");
    }

    ~MissionControlNode()
    {
        // Stop the robot when node is shutting down
        RCLCPP_INFO(this->get_logger(), "Shutting down - stopping vehicle");

        // Send STOP mode to aa10_car_yaw_control
        publishControlMode(STOP);

        // Send zero speed
        publishSpeedReal(0);

        // Also disable lane control
        publishLaneControl(false);

        // Give time for messages to be sent
        rclcpp::sleep_for(std::chrono::milliseconds(100));
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

        RCLCPP_INFO(this->get_logger(), "Real hardware subscribers created");
        RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Lidar topic: %s", lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
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
        // Parse the lane status string: "LEFT,CENTER,RIGHT"
        std::string status_str = msg->data;
        size_t first_comma = status_str.find(',');
        size_t second_comma = status_str.find(',', first_comma + 1);

        if (first_comma != std::string::npos && second_comma != std::string::npos)
        {
            std::string left_str = status_str.substr(0, first_comma);
            std::string center_str = status_str.substr(first_comma + 1, second_comma - first_comma - 1);
            std::string right_str = status_str.substr(second_comma + 1);

            lane_status_left_ = (left_str == "BLOCKED");
            lane_status_center_ = (center_str == "BLOCKED");
            lane_status_right_ = (right_str == "BLOCKED");

            // case 7에서만 로그 출력
            if (mission_flag_ == 7)
            {
                RCLCPP_INFO(this->get_logger(), "Lane Status Received: [%s, %s, %s] (Left, Center, Right)",
                            lane_status_left_ ? "BLOCKED" : "CLEAR", lane_status_center_ ? "BLOCKED" : "CLEAR",
                            lane_status_right_ ? "BLOCKED" : "CLEAR");
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
            // Main mission control logic
            double current_speed = 100.0; // Current speed based on mode
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Mission: %d | IMU: %.1f° | Speed: %.2f", mission_flag_,
                                 imu_heading_angle_degree_, current_speed);

            // Simple mission example
            switch (mission_flag_)
            {
            case 0:
                // Initialize mission - 출발 대기 (가림막 제거 대기)
                publishLaneControl(false);
                publishControlMode(STEER_CONTROL); // Direct steering control mode
                publishSteeringAngle(0.0);
                publishSpeedReal(0);
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 0: Waiting for barrier removal...");

                // Encoder 리셋 (한 번만 호출)
                if (!encoder_reset_requested_)
                {
                    // ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger 명령 실행
                    system("ros2 service call /car_control/reset_encoder std_srvs/srv/Trigger &");
                    RCLCPP_INFO(this->get_logger(), "Encoder reset requested");
                    encoder_reset_requested_ = true;
                }

                // IMU 캘리브레이션 (출발 전 IMU 각도 평균 계산하여 offset 설정)
                if (!imu_calibrated_)
                {
                    // 먼저 IMU offset을 0으로 리셋 (한 번만)
                    if (!imu_offset_reset_sent_)
                    {
                        std_msgs::msg::Float32 reset_msg;
                        reset_msg.data = 0.0;
                        imu_offset_pub_->publish(reset_msg);
                        RCLCPP_INFO(this->get_logger(), "IMU offset reset to 0° for calibration");
                        imu_offset_reset_sent_ = true;

                        // 리셋 후 잠시 대기 (IMU 노드에서 offset 적용될 때까지)
                        rclcpp::sleep_for(std::chrono::milliseconds(100));
                    }

                    // 샘플 수집 (최대 50개)
                    if (imu_calibration_samples_.size() < 50)
                    {
                        imu_calibration_samples_.push_back(imu_heading_angle_degree_);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                             "IMU calibration: collecting samples (%zu/50)...",
                                             imu_calibration_samples_.size());
                    }
                    else
                    {
                        // 평균 계산 (보정된 IMU 값의 평균)
                        double sum =
                            std::accumulate(imu_calibration_samples_.begin(), imu_calibration_samples_.end(), 0.0);
                        double avg_angle = sum / imu_calibration_samples_.size();

                        // Offset은 평균값의 음수 (보정된 값을 0으로 만들기 위해)
                        imu_angle_offset_ = -avg_angle;
                        imu_calibrated_ = true;
                        RCLCPP_INFO(this->get_logger(), "IMU calibration complete! Average angle = %.2f°, Offset = %.2f° (from %zu samples)",
                                    avg_angle, imu_angle_offset_, imu_calibration_samples_.size());

                        // Publish offset to IMU node
                        std_msgs::msg::Float32 offset_msg;
                        offset_msg.data = imu_angle_offset_;
                        imu_offset_pub_->publish(offset_msg);
                        RCLCPP_INFO(this->get_logger(), "Published IMU offset (%.2f°) to IMU node", imu_angle_offset_);
                    }
                }

                // 가림막이 제거되면 mission_flag를 1로 변경
                // Obstacle detect 노드로부터 거리 정보 받아서 판단
                // obstacle_detected_ == false: 장애물 없음 (가림막 제거됨)
                // 또는 obstacle_distance_ > 임계값: 가림막이 충분히 멀어짐
                if (imu_calibrated_ && (!obstacle_detected_ || obstacle_distance_ > 0.5))
                {
                    // IMU 캘리브레이션 완료 AND 가림막이 없거나 50cm 이상 멀어지면 출발
                    RCLCPP_INFO(this->get_logger(), "Barrier removed (detected=%s, distance=%.2fm)",
                                obstacle_detected_ ? "true" : "false", obstacle_distance_);
                    RCLCPP_INFO(this->get_logger(), "IMU calibrated with offset=%.2f°", imu_angle_offset_);
                    transitionToNextMission(3); // Move to mission 3 (10cm forward test)
                }
                else
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Waiting... (IMU cal=%s, distance=%.2fm)", imu_calibrated_ ? "OK" : "...",
                                         obstacle_distance_);
                }
                break;

            case 1:
                // AI 라인 검출 주행
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 1: Lane following with AI line detection");

                // Lane control 활성화
                publishLaneControl(true);
                publishControlMode(LANE_CONTROL); // Vision control mode
                publishSpeedReal(150);            // 주행 속도 설정

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

                if ((this->now() - mission2_stop_time).seconds() > 0.5)
                {
                    mission2_stopped = false; // Reset for next time
                    RCLCPP_INFO(this->get_logger(), "Mission 2 complete - Moving to next");
                    transitionToNextMission(3);
                }
                break;

            case 3:
            {
                // 0.05m/s (50mm/s) 속도로 10cm 전진 후 정지
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 3: Moving 10cm forward at 50 mm/s");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL); // Direct steering control mode
                publishSteeringAngle(0.0);         // Set steering to 0 degrees (straight)

                // 시작 위치 저장 (한 번만)
                if (!mission3_position_sent_)
                {
                    mission3_start_position_mm_ = encoder_position_mm_;
                    mission3_position_sent_ = true;

                    RCLCPP_INFO(this->get_logger(), "Mission 3 started at position: %.1fmm", encoder_position_mm_);
                }

                // 현재까지 이동한 거리
                double traveled_distance = encoder_position_mm_ - mission3_start_position_mm_;

                // 목표: 100mm (10cm) 이동
                if (traveled_distance < 100.0)
                {
                    // 아직 목표 거리에 도달하지 않음 - 속도 50mm/s로 전진
                    publishSpeedReal(130); // 50mm/s = 0.05m/s
                }
                else
                {
                    // 목표 거리 도달 - 정지
                    publishSpeedReal(0);

                    RCLCPP_INFO(this->get_logger(), "Mission 3 complete: Traveled %.1fmm, final position: %.1fmm",
                                traveled_distance, encoder_position_mm_);
                    mission3_position_sent_ = false; // Reset for next time
                    transitionToNextMission(4);
                }

                // 진행 상황 모니터링
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Mission 3: Traveled %.1f/100.0 mm, speed: %.1f mm/s", traveled_distance,
                                     encoder_speed_mms_);
                break;
            }

            case 4:
            {
                // 장애물 감지 - 현재 각도 저장하고 장애물 유무에 따라 분기
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 4: Checking obstacle and saving current angle");

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);
                publishSteeringAngle(0.0);
                publishSpeedReal(0); // 정지 상태에서 판단

                // 현재 IMU 각도 저장 (한 번만)
                if (!mission4_angle_saved_)
                {
                    mission4_saved_angle_ = imu_heading_angle_degree_;
                    mission4_angle_saved_ = true;
                    RCLCPP_INFO(this->get_logger(), "Mission 4: Saved reference angle = %.1f°", mission4_saved_angle_);
                }

                // 장애물 감지 여부에 따라 분기
                if (!obstacle_detected_ || obstacle_distance_ > 0.5)
                {
                    // 장애물 없음 -> Mission 5 (직진 30cm)
                    mission4_angle_saved_ = false; // Reset for next time
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 4: No obstacle detected (distance=%.2fm) - Going straight 30cm",
                                obstacle_distance_);
                    transitionToNextMission(5);
                }
                else
                {
                    // 장애물 있음 -> Mission 6 (yaw -30도로 회피하며 30cm)
                    mission4_angle_saved_ = false; // Reset for next time
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 4: Obstacle detected (distance=%.2fm) - Avoiding with yaw -30°",
                                obstacle_distance_);
                    transitionToNextMission(6);
                }
                break;
            }

            case 5:
            {
                // 장애물 없음 - 직진 30cm 이동 후 정지
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 5: Moving straight 30cm (no obstacle)");

                publishLaneControl(false);
                publishControlMode(IMU_CONTROL);                 // IMU control mode
                publishTargetAngle(mission4_saved_angle_);       // Maintain saved angle (straight)
                publishSteeringAngle(mission4_saved_angle_);

                // 시작 위치 저장 (한 번만)
                if (!mission5_position_sent_)
                {
                    mission5_start_position_mm_ = encoder_position_mm_;
                    mission5_position_sent_ = true;
                    RCLCPP_INFO(this->get_logger(), "Mission 5 started at position: %.1fmm, target angle: %.1f°",
                                encoder_position_mm_, mission4_saved_angle_);
                }

                // 현재까지 이동한 거리
                double traveled_distance = encoder_position_mm_ - mission5_start_position_mm_;

                // 목표: 300mm (30cm) 이동
                if (traveled_distance < 300.0)
                {
                    publishSpeedReal(200); // 50mm/s
                }
                else
                {
                    publishSpeedReal(0);
                    RCLCPP_INFO(this->get_logger(), "Mission 5 complete: Traveled %.1fmm straight", traveled_distance);
                    mission5_position_sent_ = false;
                    transitionToNextMission(7);
                }

                // 진행 상황 모니터링
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Mission 5: Traveled %.1f/300.0 mm, angle: %.1f°", traveled_distance,
                                     imu_heading_angle_degree_);
                break;
            }

            case 6:
            {
                // 장애물 있음 - yaw -30도로 회피하며 30cm 이동 후 정지
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 6: Moving 30cm with yaw -30° (avoiding obstacle)");

                publishLaneControl(false);
                publishControlMode(IMU_CONTROL); // IMU control mode

                // 목표 각도: 저장된 각도 - 30도
                double target_angle = mission4_saved_angle_ - 30.0;
                // 0-360 범위로 정규화
                while (target_angle < 0.0)
                    target_angle += 360.0;
                while (target_angle >= 360.0)
                    target_angle -= 360.0;

                publishTargetAngle(target_angle);

                // 시작 위치 저장 (한 번만)
                if (!mission6_position_sent_)
                {
                    mission6_start_position_mm_ = encoder_position_mm_;
                    mission6_position_sent_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "Mission 6 started at position: %.1fmm, reference: %.1f°, target: %.1f° (yaw -30°)",
                                encoder_position_mm_, mission4_saved_angle_, target_angle);
                }

                // 현재까지 이동한 거리
                double traveled_distance = encoder_position_mm_ - mission6_start_position_mm_;

                // 목표: 300mm (30cm) 이동
                if (traveled_distance < 300.0)
                {
                    publishSpeedReal(200); // 50mm/s
                }
                else
                {
                    publishSpeedReal(0);
                    RCLCPP_INFO(this->get_logger(), "Mission 6 complete: Traveled %.1fmm with yaw -30°",
                                traveled_distance);
                    mission6_position_sent_ = false;
                    transitionToNextMission(7);
                }

                // 진행 상황 모니터링
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                     "Mission 6: Traveled %.1f/300.0 mm, target: %.1f°, current: %.1f°",
                                     traveled_distance, target_angle, imu_heading_angle_degree_);
                break;
            }

            case 7:
                // Mission 5 or 6 완료 후 최종 정지
                RCLCPP_INFO_ONCE(this->get_logger(), "Mission 7: Final stop");

                publishLaneControl(false);
                publishControlMode(STOP);
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

    void publishObstacleEnable(bool enable)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = enable;
        obstacle_enable_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Obstacle detection: %s", enable ? "ENABLED" : "DISABLED");
    }

    // ==================== Parameters ====================

    // ==================== Sensor Data ====================
    double imu_heading_angle_degree_;      // Current IMU heading angle in degrees (0-360)
    double lidar_front_obstacle_distance_; // Front obstacle distance from lidar (meters)
    double stop_line_position_;            // Stop line position from vision (pixels)
    bool lane_status_left_;                // Left lane obstacle status (false = CLEAR, true = BLOCKED)
    bool lane_status_center_;              // Center lane obstacle status (false = CLEAR, true = BLOCKED)
    bool lane_status_right_;               // Right lane obstacle status (false = CLEAR, true = BLOCKED)

    // Obstacle detection data
    double obstacle_distance_; // Front obstacle distance (meters)
    bool obstacle_detected_;   // Obstacle detected flag

    // IMU calibration data
    std::vector<double> imu_calibration_samples_; // IMU samples for calibration
    double imu_angle_offset_;                     // IMU angle offset for calibration
    bool imu_calibrated_;                         // IMU calibration complete flag
    bool imu_offset_reset_sent_;                  // Flag to track if offset reset was sent

    // Encoder data (from powerpack driver)
    double encoder_position_mm_; // Vehicle position in millimeters (accumulated)
    double encoder_speed_mms_;   // Vehicle speed in mm/s

    // Mission 3 control data
    double mission3_start_position_mm_; // Starting position for mission 3
    bool mission3_position_sent_;       // Flag to track if position command was sent
    bool encoder_reset_requested_;      // Flag to track if encoder reset was requested

    // Mission 4, 5, 6 control data
    double mission4_saved_angle_;       // Saved IMU angle at mission 4 (reference angle)
    bool mission4_angle_saved_;         // Flag to track if angle was saved
    double mission5_start_position_mm_; // Starting position for mission 5
    bool mission5_position_sent_;       // Flag to track if position command was sent
    double mission6_start_position_mm_; // Starting position for mission 6
    bool mission6_position_sent_;       // Flag to track if position command was sent

    // ==================== Mission Control ====================
    int mission_flag_;       // Current mission state
    int start_mission_flag_; // Mission to start from (set by GUI)
    int end_mission_flag_;   // Mission to end at (set by GUI)
    bool run_flag_;          // Race start/stop flag

    // ==================== Subscribers - Real Hardware ====================
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_real_sub_;

    // ==================== Subscribers - Common ====================
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_flag_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stop_line_position_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr obstacle_distance_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_detected_sub_;
    rclcpp::Subscription<amap_powerpack_single_driver::msg::EncoderStatus>::SharedPtr encoder_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr start_mission_flag_sub_; // Start mission config from GUI
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr end_mission_flag_sub_;   // End mission config from GUI

    // ==================== Publishers ====================
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr yaw_control_mode_pub_; // Steering control mode
    rclcpp::Publisher<amap_powerpack_single_driver::msg::TargetSpeed>::SharedPtr
        car_control_speed_pub_; // Speed command (Real hardware)
    rclcpp::Publisher<amap_powerpack_single_driver::msg::SteeringAngle>::SharedPtr
        car_control_steering_pub_;                                             // Steering angle command
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lane_control_flag_pub_;  // Lane control enable/disable
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_angle_pub_;    // Target angle for IMU control
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        target_angular_velocity_pub_;                                       // Target angular velocity for STEER_CONTROL
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_enable_pub_; // Obstacle detection enable/disable
    rclcpp::Publisher<amap_powerpack_single_driver::msg::TargetPositionRelative>::SharedPtr
        target_position_relative_pub_;                                     // Relative position control
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr imu_offset_pub_;   // IMU angle offset
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mission_status_pub_; // Mission status for GUI

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