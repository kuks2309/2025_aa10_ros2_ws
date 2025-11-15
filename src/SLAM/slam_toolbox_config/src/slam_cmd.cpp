#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

int main(int argc, char * argv[])
{
    if (argc < 2) {
        std::cout << "Usage: slam_cmd [start|stop|reset|status]" << std::endl;
        return 1;
    }
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("slam_cmd_client");
    
    auto publisher = node->create_publisher<std_msgs::msg::String>("/slam_command", 10);
    
    // 노드가 준비될 때까지 대기
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    auto message = std_msgs::msg::String();
    message.data = argv[1];
    
    std::cout << "📡 Sending command: " << message.data << std::endl;
    publisher->publish(message);
    
    rclcpp::shutdown();
    return 0;
}