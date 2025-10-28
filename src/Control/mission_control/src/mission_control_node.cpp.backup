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
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

#define RAD2DEG(x) ((x) * 180. / M_PI)
#define DEG2RAD(x) ((x) / 180. * M_PI)

#define STOP          0   
#define IMU_CONTROL   1
#define LANE_CONTROL  2
#define CONE_MAZE_CONTROL  3
#define STEER_CONTROL 4
#define WALL_FOLLOWING 5
#define OBSTACLE_DETECT 6

class MissionControlNode : public rclcpp::Node
{
  public:
    MissionControlNode() : Node("mission_control_node")
    {
        // Parameters for real hardware
        this->declare_parameter("imu_topic", "/handsfree/imu_yaw_degree");
        this->declare_parameter("lidar_topic", "/scan");
        this->declare_parameter("camera_topic", "/camera/image_raw");

        // Initialize variables
        imu_heading_angle_degree_ = 0.0;
        lidar_front_obstacle_distance_ = 0.0;
        stop_line_position_ = 0.0;
        mission_flag_ = 0;
        run_flag_ = false;
        lane_status_left_ = false;   // false = CLEAR, true = BLOCKED
        lane_status_center_ = false; // false = CLEAR, true = BLOCKED
        lane_status_right_ = false;  // false = CLEAR, true = BLOCKED

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
            "/lane_status", 10,
            std::bind(&MissionControlNode::laneStatusCallback, this, std::placeholders::_1));

        // Publishers
        yaw_control_mode_pub_ =
            this->create_publisher<std_msgs::msg::Int8>("/Car_Control_Cmd/steering_control_mode", 5);
        lane_control_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/flag/lane_control_set", 5);
        target_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/Car_Control_Cmd/Target_Angle", 5);
        target_angular_velocity_pub_ = this->create_publisher<std_msgs::msg::Float32>("/target_angular_velocity", 5);
        obstacle_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/obstacle_detect_enable", 5);

        // Create speed publishers for both modes
        car_control_speed_pub_ = this->create_publisher<std_msgs::msg::Int16>("/Car_Control_Cmd/Speed_Int16", 5);
        
        if (simulation_mode_)
        {
            // Gazebo uses Float32 vehicle_speed and cmd_vel
            vehicle_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/vehicle_speed", 5);
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);
        }

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
        if (simulation_mode_)
        {
            publishSpeedGazebo(0.0);
            // Also send zero angular velocity for STEER_CONTROL mode
            publishTargetAngularVelocity(0.0);
        }
        else
        {
            publishSpeedReal(0);
        }
        
        // Also disable lane control
        publishLaneControl(false);
        
        // Give time for messages to be sent
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

  private:
    void setupRealSubscribers()
    {
        std::string imu_topic = this->get_parameter("imu_real_topic").as_string();
        std::string lidar_topic = this->get_parameter("lidar_real_topic").as_string();
        std::string camera_topic = this->get_parameter("camera_real_topic").as_string();

        // IMU subscriber for real hardware (Float32 yaw degree)
        imu_real_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            imu_topic, 10, std::bind(&MissionControlNode::imuRealCallback, this, std::placeholders::_1));

        // Lidar subscriber for real hardware (sensor_msgs/LaserScan)
        lidar_real_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&MissionControlNode::lidarRealCallback, this, std::placeholders::_1));

        // Camera subscriber for real hardware (sensor_msgs/Image)
        camera_real_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10, std::bind(&MissionControlNode::cameraRealCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Real hardware subscribers created");
        RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Lidar topic: %s", lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
    }

    void setupGazeboSubscribers()
    {
        std::string imu_topic = this->get_parameter("imu_gazebo_topic").as_string();
        std::string lidar_topic = this->get_parameter("lidar_gazebo_topic").as_string();
        std::string camera_topic = this->get_parameter("camera_gazebo_topic").as_string();

        // IMU subscriber for Gazebo (sensor_msgs/Imu)
        imu_gazebo_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 10, std::bind(&MissionControlNode::imuGazeboCallback, this, std::placeholders::_1));

        // Lidar subscriber for Gazebo (sensor_msgs/LaserScan)
        lidar_gazebo_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&MissionControlNode::lidarGazeboCallback, this, std::placeholders::_1));

        // Camera subscriber for Gazebo (sensor_msgs/Image)
        camera_gazebo_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10, std::bind(&MissionControlNode::cameraGazeboCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Gazebo subscribers created");
        RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Lidar topic: %s", lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
    }

    // Real hardware callbacks
    void imuRealCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        imu_heading_angle_degree_ = msg->data;
        if (imu_heading_angle_degree_ < 0)
        {
            imu_heading_angle_degree_ += 360.0;
        }
        RCLCPP_DEBUG(this->get_logger(), "Real IMU yaw: %.3f degrees", imu_heading_angle_degree_);
    }

    void lidarRealCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Extract front distance from LaserScan (front beam at index 0)
        if (!msg->ranges.empty())
        {
            size_t front_index = 0;  // 0도 = 전방
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

    // Gazebo callbacks
    void imuGazeboCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Convert quaternion to yaw angle in degrees
        double yaw_rad = tf2::getYaw(msg->orientation);
        imu_heading_angle_degree_ = RAD2DEG(yaw_rad);

        // Normalize to 0-360 degrees
        if (imu_heading_angle_degree_ < 0)
        {
            imu_heading_angle_degree_ += 360.0;
        }
        RCLCPP_DEBUG(this->get_logger(), "Gazebo IMU yaw: %.3f degrees", imu_heading_angle_degree_);
    }

    void lidarGazeboCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Extract front distance from LaserScan (front beam at index 0)
        if (!msg->ranges.empty())
        {
            size_t front_index = 0;  // 0도 = 전방
            lidar_front_obstacle_distance_ = msg->ranges[front_index];

            // Handle inf/nan values
            if (std::isinf(lidar_front_obstacle_distance_) || std::isnan(lidar_front_obstacle_distance_))
            {
                lidar_front_obstacle_distance_ = msg->range_max;
            }
            
        }
    }

    void cameraGazeboCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Camera image received - for now just log
        static int camera_count = 0;
        if (++camera_count % 30 == 0)
        { // Log every 30 frames (~1 second at 30Hz)
            RCLCPP_DEBUG(this->get_logger(), "Gazebo Camera image received: %dx%d, encoding: %s", msg->width, msg->height,
                         msg->encoding.c_str());
        }
    }

    void runFlagCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        run_flag_ = msg->data;
        RCLCPP_DEBUG(this->get_logger(), "Race run flag: %s", run_flag_ ? "true" : "false");
    }

    void stopLinePositionCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        stop_line_position_ = msg->data;
        // 특정 mission_flag에서만 로그 출력 (case 1, 3, 5, 16 등 stop line이 필요한 경우)
        if (mission_flag_ == 1 || mission_flag_ == 3 || mission_flag_ == 5 || mission_flag_ == 16) {
            RCLCPP_INFO(this->get_logger(), "Stop line position received: %.3f", stop_line_position_);
        }
    }

    void laneStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // Parse the lane status string: "LEFT,CENTER,RIGHT"
        std::string status_str = msg->data;
        size_t first_comma = status_str.find(',');
        size_t second_comma = status_str.find(',', first_comma + 1);
        
        if (first_comma != std::string::npos && second_comma != std::string::npos) {
            std::string left_str = status_str.substr(0, first_comma);
            std::string center_str = status_str.substr(first_comma + 1, second_comma - first_comma - 1);
            std::string right_str = status_str.substr(second_comma + 1);
            
            lane_status_left_ = (left_str == "BLOCKED");
            lane_status_center_ = (center_str == "BLOCKED");
            lane_status_right_ = (right_str == "BLOCKED");
            
            // case 7에서만 로그 출력
            if (mission_flag_ == 7) {
                RCLCPP_INFO(this->get_logger(), "Lane Status Received: [%s, %s, %s] (Left, Center, Right)", 
                           lane_status_left_ ? "BLOCKED" : "CLEAR", 
                           lane_status_center_ ? "BLOCKED" : "CLEAR", 
                           lane_status_right_ ? "BLOCKED" : "CLEAR");
            }
        }
    }

    void controlLoop()
    {
        auto current_time = this->now();
        
        if (run_flag_)
        {
            // Main mission control logic
            double current_speed = simulation_mode_ ? 0.15 : 100.0; // Current speed based on mode
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Mission: %d | IMU: %.1f° | Speed: %.2f | Mode: %s", mission_flag_,
                                 imu_heading_angle_degree_, current_speed,
                                 simulation_mode_ ? "Gazebo" : "Real");

            // Simple mission example
            switch (mission_flag_)
            {
            case 0:
                // Initialize mission
                 mission_flag_ = 6;
                 break;
            case 1:
            {
                static rclcpp::Time case1_stop_line_detected_time;
                static bool case1_stop_line_detected = false;
                
                // Lane following with stop line detection, 3 sec wait, go to case 2
                publishLaneControl(true);
                publishControlMode(LANE_CONTROL);

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.20); // 0.20 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(100); // 100 for real hardware
                }

                // Debug: Log stop line position continuously
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                    "Case 1: Current stop_line_position: %.1f (threshold: 235)", stop_line_position_);

                if (stop_line_position_ >= 235)
                {
                    // Save detection time when first detected
                    if (!case1_stop_line_detected)
                    {
                        case1_stop_line_detected_time = current_time;
                        case1_stop_line_detected = true;
                        RCLCPP_INFO(this->get_logger(), "Stop line detected at position: %.1f, waiting 3 seconds...", stop_line_position_);
                    }
                    
                    // Stop when stop line detected

                    publishLaneControl(false);
                    publishControlMode(STOP);

                    if (simulation_mode_)
                    {
                        publishSpeedGazebo(0.0);
                    }
                    else
                    {
                        publishSpeedReal(0);
                    }
                    
                    // Check if 3 seconds have passed
                    if ((current_time - case1_stop_line_detected_time).seconds() >= 3.0)
                    {
                        mission_flag_ = 2;
                        case1_stop_line_detected = false;  // Reset for next time
                        RCLCPP_INFO(this->get_logger(), "3 seconds passed, proceeding to case 2");
                    }
                }
            }
                break;
            
            case 2:
            
                // 
            
                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);
                publishTargetAngularVelocity(-0.2); // 0.5 rad/s 각속도

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.2); // 0.2 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(150); // 150 for real hardware
                }

                // 여기서 heading angle이 0도 근처이면 mission_flag=3
                if (imu_heading_angle_degree_ <= 18.0 || imu_heading_angle_degree_ >= 342.0)
                {
                    mission_flag_ = 3;
                    RCLCPP_INFO(this->get_logger(), "Heading angle close to 0° (%.1f°), proceeding to case 3", imu_heading_angle_degree_);
                }

                break; 
            
            
                
            case 3:
                // Lane following with stop line detection, 3 sec wait, go to case 2
                publishLaneControl(true);
                publishControlMode(LANE_CONTROL);

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.2); // 0.2 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(100); // 100 for real hardware
                }
                
                if (imu_heading_angle_degree_ >= 50.0 )
                {
                    mission_flag_ = 4;
                    RCLCPP_INFO(this->get_logger(), "Heading angle close to 0° (%.1f°), proceeding to case 3", imu_heading_angle_degree_);
                }
            

                break;

            case 4:

                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);
                publishTargetAngularVelocity(-0.2); // 0.5 rad/s 각속도

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.2); // 0.2 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(150); // 150 for real hardware
                }

                // 여기서 heading angle이 0도 근처이면 mission_flag=3
                if (imu_heading_angle_degree_ <= 12.0 )
                {
                    mission_flag_ = 5;
                    RCLCPP_INFO(this->get_logger(), "Heading angle close to 0° (%.1f°), proceeding to case 3", imu_heading_angle_degree_);
                }

                break; 

            case 5:
                // Lane following with stop line detection, 3 sec wait, go to case 2
                publishLaneControl(true);
                publishControlMode(LANE_CONTROL);

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.2); // 0.2 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(100); // 100 for real hardware
                }

                if (stop_line_position_ >= 240)
                {
                    mission_flag_ = 6;
                }
                break;    


                case 6:
                 
                // IMU control
                publishLaneControl(false);
                publishControlMode(IMU_CONTROL);
                publishTargetAngle(0.0); // Set target angle to 90 degrees

                
                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.15); // 0.15 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(100); // 100 for real hardware
                }
                /*
                if (imu_heading_angle_degree_ <= 2.0 )
                {
                    mission_flag_ = 7;
                    RCLCPP_INFO(this->get_logger(), "Heading angle close to 0° (%.1f°), proceeding to case 7", imu_heading_angle_degree_);
                }
                */

                if (lidar_front_obstacle_distance_ <= 2.4)
                {
                    mission_flag_ = 7;
                    RCLCPP_INFO(this->get_logger(), "Front Obstacle distance: %.1fm, proceeding to case 7", lidar_front_obstacle_distance_);
                }
                
                
                break;

                case 7: // obstacle detect 
                {
                    static rclcpp::Time obstacle_enable_time;
                    static bool obstacle_enabled = false;
                    static bool lane_selected = false;
                    
                    publishLaneControl(false);
                    publishControlMode(OBSTACLE_DETECT);

                    // Control mode 6 (OBSTACLE_DETECT) automatically enables obstacle detection
                    if (!obstacle_enabled) {
                        obstacle_enable_time = current_time;
                        obstacle_enabled = true;
                        RCLCPP_INFO(this->get_logger(), "Obstacle detection mode set, waiting 0.5 seconds...");
                    }
                    
                    if (simulation_mode_)
                    {
                        publishSpeedGazebo(0.0);
                    }
                    else
                    {
                        publishSpeedReal(0);
                    }
                    
                    // 0.5초 정도 기다리고 
                    if ((current_time - obstacle_enable_time).seconds() >= 0.5)
                    {
                        // lane의 상태를 읽도록 
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Lane Status: [%s, %s, %s] (Left, Center, Right)", 
                                   lane_status_left_ ? "BLOCKED" : "CLEAR", 
                                   lane_status_center_ ? "BLOCKED" : "CLEAR", 
                                   lane_status_right_ ? "BLOCKED" : "CLEAR");
                        
                        // 여기서 empty 라인을 찾음 좌우만 선택함
                        // 둘다 비었을때는 왼쪽 라인 선택
                        if (!lane_selected) 
                        {
                            if (!lane_status_left_) 
                            {  // 왼쪽 레인이 비어있으면
                                RCLCPP_INFO(this->get_logger(), "Selecting LEFT lane - CLEAR");
                                mission_flag_ = 8;  // 왼쪽 레인으로 진행
                                lane_selected = true;
                            } 
                            else if (!lane_status_right_) 
                            {  // 오른쪽 레인이 비어있으면
                                RCLCPP_INFO(this->get_logger(), "Selecting RIGHT lane - CLEAR");
                                mission_flag_ = 12;  // 오른쪽 레인으로 진행
                                lane_selected = true;
                            } 
                            else 
                            {
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                           "Both lanes BLOCKED - waiting");
                            }
                        }
                    }
                }
                break;

            case 8: // 왼쪽 레인으로 이동
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Case 8: Moving to LEFT lane");
                
                publishLaneControl(false);
                publishObstacleEnable(false);
                publishControlMode(STEER_CONTROL);
                publishTargetAngularVelocity(0.5); // 왼쪽으로 회전 (양수)
                
                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.2);
                }
                else
                {
                    publishSpeedReal(100);
                }

                if((imu_heading_angle_degree_ <= 50.0 ) &&(imu_heading_angle_degree_ >= 38.0 ))
                {
                    mission_flag_ = 9;
                    RCLCPP_INFO(this->get_logger(), "Heading angle close to 0° (%.1f°), proceeding to case 9", imu_heading_angle_degree_);
                }
                
                // TODO: 왼쪽 레인 진입 완료 조건 추가
                // 예: IMU 각도나 일정 시간 후 다음 state로 전환
                break;

            case 9: // imu로 수평 각도 유지 기능 
            
                publishLaneControl(false);
                publishControlMode(IMU_CONTROL);
                publishTargetAngle(0.0);  // target angle = 0 도 

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.15);
                }
                else
                {
                    publishSpeedReal(100);
                }

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Case 10: IMU Control - Target: 0°, Current: %.1f°", 
                           imu_heading_angle_degree_);
                // 0도 근처에 도달했는지 확인 (0~2도 또는 358~360도)
                if (imu_heading_angle_degree_ <= 2.0 || imu_heading_angle_degree_ >= 358.0)
                {
                    mission_flag_ = 10;
                    RCLCPP_INFO(this->get_logger(), "Reached target angle (%.1f°), proceeding to case 10", imu_heading_angle_degree_);
                }

                break;    
            
            case 10: // 중앙 레인으로 이동
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Case 9: Moving to RIGHT lane");
                
                publishLaneControl(false);
                publishObstacleEnable(false);
                publishControlMode(STEER_CONTROL);
                publishTargetAngularVelocity(-0.4); // 오른쪽으로 회전 (음수)
                
                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.15);
                }
                else
                {
                    publishSpeedReal(100);
                }

                // 오른쪽으로 30도 회전 후 (330도 근처) case 15으로 전환
                if (imu_heading_angle_degree_ <= 325.0 && imu_heading_angle_degree_ >= 300.0)
                {
                    mission_flag_ = 15; // 0도 imu 유지
                    RCLCPP_INFO(this->get_logger(), "Center turn completed at %.1f°, proceeding to case 10", imu_heading_angle_degree_);
                }
                
                // TODO: 오른쪽 레인 진입 완료 조건 추가
                // 예: IMU 각도나 일정 시간 후 다음 state로 전환
                break;






                
            case 12: // 오른쪽 레인으로 이동
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Case 9: Moving to RIGHT lane");
                
                publishLaneControl(false);
                publishObstacleEnable(false);
                publishControlMode(STEER_CONTROL);
                publishTargetAngularVelocity(-0.4); // 오른쪽으로 회전 (음수)
                
                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.15);
                }
                else
                {
                    publishSpeedReal(100);
                }

                // 오른쪽으로 30도 회전 후 (330도 근처) case 10으로 전환
                if (imu_heading_angle_degree_ <= 325.0 && imu_heading_angle_degree_ >= 300.0)
                {
                    mission_flag_ = 13;
                    RCLCPP_INFO(this->get_logger(), "Right turn completed at %.1f°, proceeding to case 10", imu_heading_angle_degree_);
                }
                
                // TODO: 오른쪽 레인 진입 완료 조건 추가
                // 예: IMU 각도나 일정 시간 후 다음 state로 전환
                break;

            case 13: // imu로 각도 유지 기능 
            
                publishLaneControl(false);
                publishControlMode(IMU_CONTROL);
                publishTargetAngle(0.0);  // target angle = 0 도 

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.15);
                }
                else
                {
                    publishSpeedReal(100);
                }

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Case 10: IMU Control - Target: 0°, Current: %.1f°", 
                           imu_heading_angle_degree_);
                // 0도 근처에 도달했는지 확인 (0~2도 또는 358~360도)
                if (imu_heading_angle_degree_ <= 2.0 || imu_heading_angle_degree_ >= 358.0)
                {
                    mission_flag_ = 14;
                    RCLCPP_INFO(this->get_logger(), "Reached target angle (%.1f°), proceeding to case 11", imu_heading_angle_degree_);
                }

                break;

            case 14: // 다시 차선 복귀

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Case 8: Return to Center lane");
                
                publishLaneControl(false);
                publishControlMode(STEER_CONTROL);
                publishTargetAngularVelocity(0.4); // 왼쪽으로 회전 (양수)
                
                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.15);
                }
                else
                {
                    publishSpeedReal(100);
                }

                if ( (imu_heading_angle_degree_ >= 35.0 ) &&(imu_heading_angle_degree_ <= 50.0 ) )
                {
                    mission_flag_ = 15;
                    RCLCPP_INFO(this->get_logger(), "Heading angle close to 0° (%.1f°), proceeding to case 3", imu_heading_angle_degree_);
                }
                
                // TODO: 왼쪽 레인 진입 완료 조건 추가
                // 예: IMU 각도나 일정 시간 후 다음 state로 전환
                break;

             case 15: // imu로 각도 유지 기능 
            
                publishLaneControl(false);
                publishControlMode(IMU_CONTROL);
                publishTargetAngle(0.0);  // target angle = 0 도 

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.15);
                }
                else
                {
                    publishSpeedReal(100);
                }

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Case 12: IMU Control - Target: 0°, Current: %.1f°", 
                           imu_heading_angle_degree_);
                // 0도 근처에 도달했는지 확인 (0~2도 또는 358~360도)
                if (imu_heading_angle_degree_ <= 2.0 || imu_heading_angle_degree_ >= 358.0)
                {
                    mission_flag_ = 16;
                    RCLCPP_INFO(this->get_logger(), "Reached target angle (%.1f°), proceeding to case 11", imu_heading_angle_degree_);
                }

                break;        
            
             case 16:
                // Lane following with stop line detection, 3 sec wait, go to case 2
                publishLaneControl(true);
                publishControlMode(LANE_CONTROL);

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.2); // 0.2 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(100); // 100 for real hardware
                }

                // Debug: Log stop line position continuously
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                    "Case 16: Current stop_line_position: %.1f (threshold: 240)", stop_line_position_);

                if (stop_line_position_ >= 180)
                {
                    RCLCPP_INFO(this->get_logger(), "Stop line detected at position: %.1f, proceeding to case 17", stop_line_position_);
                    mission_flag_ = 17;
                }
                break;        

            case 17: // 자세 제어  
                publishLaneControl(false);
                publishControlMode(IMU_CONTROL);
                publishTargetAngle(0.0);  // target angle = 0 도 

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.15);
                }
                else
                {
                    publishSpeedReal(100);
                }

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Case 12: IMU Control - Target: 0°, Current: %.1f°", 
                           imu_heading_angle_degree_);
                // 0도 근처에 도달했는지 확인 (0~2도 또는 358~360도)
                if (imu_heading_angle_degree_ <= 2.0 || imu_heading_angle_degree_ >= 358.0)
                {
                    mission_flag_ = 18;
                    RCLCPP_INFO(this->get_logger(), "Reached target angle (%.1f°), proceeding to case 18", imu_heading_angle_degree_);
                }  
                

                break;
            case 18: // lidar cone maze
                publishLaneControl(false);
                publishControlMode(CONE_MAZE_CONTROL);

                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.2); // 0.2 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(100); // 100 for real hardware
                }

                break;
            


            case 19:
                publishLaneControl(false);
                publishControlMode(STOP);
                if (simulation_mode_)
                {
                    publishSpeedGazebo(0.0); // 0.2 m/s for Gazebo
                }
                else
                {
                    publishSpeedReal(0); // 100 for real hardware
                }
            
                break;

            default:
                break;
            }



        }
        else
        {
            // Stop all control
            publishLaneControl(false);
            if (simulation_mode_)
            {
                publishSpeedGazebo(0.0);
            }
            else
            {
                publishSpeedReal(0);
            }
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
        std_msgs::msg::Int16 msg;
        msg.data = speed;
        car_control_speed_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Real speed command: %d", speed);
    }

    void publishSpeedGazebo(double linear_speed)
    {
        // Publish Float32 vehicle_speed for aa10_car_yaw_control
        if (vehicle_speed_pub_)
        {
            std_msgs::msg::Float32 speed_msg;
            speed_msg.data = linear_speed;
            vehicle_speed_pub_->publish(speed_msg);
            RCLCPP_DEBUG(this->get_logger(), "Publishing vehicle_speed: %.3f m/s", linear_speed);
        }
        
        // Also publish geometry_msgs/Twist cmd_vel for visualization
        if (cmd_vel_pub_)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = linear_speed;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = 0.0; // Steering will be handled by aa10_car_yaw_control
            cmd_vel_pub_->publish(cmd_vel);
        }
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
    bool simulation_mode_; // true: Gazebo simulation, false: Real hardware

    // ==================== Sensor Data ====================
    double imu_heading_angle_degree_; // Current IMU heading angle in degrees (0-360)
    double lidar_front_obstacle_distance_;  // Front obstacle distance from lidar (meters)
    double stop_line_position_;       // Stop line position from vision (pixels)
    bool lane_status_left_;           // Left lane obstacle status (false = CLEAR, true = BLOCKED)
    bool lane_status_center_;         // Center lane obstacle status (false = CLEAR, true = BLOCKED)
    bool lane_status_right_;          // Right lane obstacle status (false = CLEAR, true = BLOCKED)

    // ==================== Mission Control ====================
    int mission_flag_; // Current mission state
    bool run_flag_;    // Race start/stop flag

    // ==================== Subscribers - Real Hardware ====================
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr imu_real_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_real_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_real_sub_;

    // ==================== Subscribers - Gazebo Simulation ====================
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_gazebo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_gazebo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_gazebo_sub_;

    // ==================== Subscribers - Common ====================
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr run_flag_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr stop_line_position_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_status_sub_;

    // ==================== Publishers ====================
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr yaw_control_mode_pub_;   // Steering control mode
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr car_control_speed_pub_; // Speed command (Real hardware)
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vehicle_speed_pub_;   // Speed command (Gazebo simulation)
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;      // Speed command (Gazebo visualization)
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lane_control_flag_pub_;  // Lane control enable/disable
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_angle_pub_;    // Target angle for IMU control
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_angular_velocity_pub_; // Target angular velocity for STEER_CONTROL
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_enable_pub_;    // Obstacle detection enable/disable

    // ==================== Timer ====================
    rclcpp::TimerBase::SharedPtr timer_; // Main control loop timer (10Hz)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionControlNode>());
    rclcpp::shutdown();
    return 0;
}