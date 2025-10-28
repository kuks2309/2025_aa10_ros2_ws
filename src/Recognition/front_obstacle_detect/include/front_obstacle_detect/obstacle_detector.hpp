#ifndef FRONT_OBSTACLE_DETECT__OBSTACLE_DETECTOR_HPP_
#define FRONT_OBSTACLE_DETECT__OBSTACLE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>
#include <vector>
#include <fstream>

namespace front_obstacle_detect
{

struct ROIConfig
{
  double x_min;
  double x_max;
  double y_min;
  double y_max;
};

struct DetectionConfig
{
  int min_points_threshold;
  double distance_threshold;
  double publish_rate_hz;
};

class ObstacleDetector : public rclcpp::Node
{
public:
  explicit ObstacleDetector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ObstacleDetector() = default;

private:
  void loadConfig(const std::string & config_file);
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool isPointInROI(double x, double y) const;
  void publishResults(double min_distance, bool detected);
  void publishROIMarker();
  void timerCallback();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Configuration
  ROIConfig roi_config_;
  DetectionConfig detection_config_;

  // Topics
  std::string lidar_input_topic_;
  std::string distance_output_topic_;
  std::string detected_output_topic_;

  // State
  double last_min_distance_;
  bool obstacle_detected_;
};

}  // namespace front_obstacle_detect

#endif  // FRONT_OBSTACLE_DETECT__OBSTACLE_DETECTOR_HPP_
