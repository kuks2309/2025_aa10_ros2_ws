#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>

class SlamManager : public rclcpp::Node
{
public:
    SlamManager() : Node("slam_manager"), slam_pid_(-1)
    {
        // Subscribers
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/slam_command", 10,
            std::bind(&SlamManager::commandCallback, this, std::placeholders::_1));
        
        // Publishers
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/slam_status", 10);
        running_pub_ = this->create_publisher<std_msgs::msg::Bool>("/slam_running", 10);
        
        // Services
        start_service_ = this->create_service<std_srvs::srv::Empty>(
            "/slam_start", std::bind(&SlamManager::startService, this, 
            std::placeholders::_1, std::placeholders::_2));
        
        stop_service_ = this->create_service<std_srvs::srv::Empty>(
            "/slam_stop", std::bind(&SlamManager::stopService, this, 
            std::placeholders::_1, std::placeholders::_2));
        
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "/slam_reset", std::bind(&SlamManager::resetService, this, 
            std::placeholders::_1, std::placeholders::_2));
        
        // Timer for status publishing
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SlamManager::publishStatus, this));
        
        RCLCPP_INFO(this->get_logger(), "🎯 SLAM Manager started!");
        RCLCPP_INFO(this->get_logger(), "Commands:");
        RCLCPP_INFO(this->get_logger(), "  Topic: ros2 topic pub /slam_command std_msgs/String \"data: 'start|stop|reset|status'\"");
        RCLCPP_INFO(this->get_logger(), "  Service: ros2 service call /slam_start std_srvs/srv/Empty");
    }
    
    ~SlamManager()
    {
        if (slam_pid_ > 0) {
            stopSlam();
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr running_pub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    pid_t slam_pid_;
    
    void commandCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        RCLCPP_INFO(this->get_logger(), "📡 Received command: %s", command.c_str());
        
        if (command == "start" || command == "on") {
            startSlam();
        } else if (command == "stop" || command == "off") {
            stopSlam();
        } else if (command == "reset") {
            resetSlam();
        } else if (command == "status") {
            publishStatus();
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ Unknown command: %s", command.c_str());
            RCLCPP_INFO(this->get_logger(), "   Valid commands: start, stop, reset, status");
        }
    }
    
    void startService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        startSlam();
    }
    
    void stopService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                    std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        stopSlam();
    }
    
    void resetService(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        resetSlam();
    }
    
    bool startSlam()
    {
        if (isRunning()) {
            RCLCPP_WARN(this->get_logger(), "⚠️  SLAM is already running! (PID: %d)", slam_pid_);
            return false;
        }
        
        slam_pid_ = fork();
        
        if (slam_pid_ == 0) {
            // 자식 프로세스: SLAM 실행
            execl("/bin/bash", "bash", "-c",
                  "source /opt/ros/humble/setup.bash && "
                  "source /home/amap/2025_aa10_ros2_ws/install/setup.bash && "
                  "ros2 launch slam_toolbox_config online_async_launch.py",
                  (char*)NULL);
            exit(1);
        } else if (slam_pid_ > 0) {
            RCLCPP_INFO(this->get_logger(), "🚀 SLAM started! (PID: %d)", slam_pid_);
            publishStatusMessage("STARTED");
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to start SLAM!");
            publishStatusMessage("FAILED_TO_START");
            return false;
        }
    }
    
    bool stopSlam()
    {
        if (!isRunning()) {
            RCLCPP_WARN(this->get_logger(), "⚠️  SLAM is not running!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "🛑 Stopping SLAM (PID: %d)...", slam_pid_);

        // 1. Fork한 자식 프로세스 종료
        if (kill(slam_pid_, SIGTERM) == 0) {
            int status;
            waitpid(slam_pid_, &status, 0);
        }

        // 2. async_slam_toolbox_node 프로세스만 종료 (slam_manager는 제외)
        RCLCPP_INFO(this->get_logger(), "🔍 Killing all SLAM Toolbox processes...");
        system("pkill -9 -f async_slam_toolbox_node");

        usleep(500000); // 0.5초 대기

        slam_pid_ = -1;
        publishStatusMessage("STOPPED");
        RCLCPP_INFO(this->get_logger(), "✅ SLAM stopped successfully!");

        return true;
    }
    
    bool resetSlam()
    {
        RCLCPP_INFO(this->get_logger(), "🔄 Resetting SLAM...");
        
        if (isRunning()) {
            stopSlam();
            sleep(1); // 1초 대기
        }
        
        bool result = startSlam();
        if (result) {
            RCLCPP_INFO(this->get_logger(), "✅ SLAM reset completed!");
            publishStatusMessage("RESET_COMPLETED");
        }
        return result;
    }
    
    bool isRunning()
    {
        if (slam_pid_ <= 0) return false;
        
        // 프로세스가 실제로 실행 중인지 확인
        if (kill(slam_pid_, 0) == 0) {
            return true;
        } else {
            slam_pid_ = -1;
            return false;
        }
    }
    
    void publishStatus()
    {
        std::string status_msg;
        bool running = isRunning();
        
        if (running) {
            status_msg = "🟢 RUNNING (PID: " + std::to_string(slam_pid_) + ")";
        } else {
            status_msg = "🔴 STOPPED";
        }
        
        auto status = std_msgs::msg::String();
        status.data = status_msg;
        status_pub_->publish(status);
        
        auto running_msg = std_msgs::msg::Bool();
        running_msg.data = running;
        running_pub_->publish(running_msg);
    }
    
    void publishStatusMessage(const std::string& msg)
    {
        auto status = std_msgs::msg::String();
        status.data = msg;
        status_pub_->publish(status);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SlamManager>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}