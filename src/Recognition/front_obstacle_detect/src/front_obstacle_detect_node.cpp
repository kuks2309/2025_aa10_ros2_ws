#include "front_obstacle_detect/obstacle_detector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<front_obstacle_detect::ObstacleDetector>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("front_obstacle_detect"), "Exception: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
