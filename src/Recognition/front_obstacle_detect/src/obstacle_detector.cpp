#include "front_obstacle_detect/obstacle_detector.hpp"
#include <nlohmann/json.hpp>
#include <cmath>
#include <algorithm>

using json = nlohmann::json;

namespace front_obstacle_detect
{

ObstacleDetector::ObstacleDetector(const rclcpp::NodeOptions & options)
: Node("obstacle_detector", options),
  last_min_distance_(std::numeric_limits<double>::infinity()),
  obstacle_detected_(false)
{
  // Declare parameters
  this->declare_parameter<std::string>("config_file", "");

  // Get config file path
  std::string config_file = this->get_parameter("config_file").as_string();

  if (config_file.empty()) {
    RCLCPP_ERROR(this->get_logger(), "config_file parameter is required!");
    throw std::runtime_error("Missing config_file parameter");
  }

  // Load configuration
  loadConfig(config_file);

  // Create publishers
  distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    distance_output_topic_, 10);
  detected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    detected_output_topic_, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/obstacle/roi_marker", 10);

  // Create timer for ROI marker publishing (1 Hz)
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ObstacleDetector::timerCallback, this));

  // Try to create LaserScan subscriber first
  try {
    laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      lidar_input_topic_, 10,
      std::bind(&ObstacleDetector::laserScanCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to LaserScan topic: %s", lidar_input_topic_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Failed to subscribe to LaserScan, trying PointCloud2...");

    // If LaserScan fails, try PointCloud2
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_input_topic_, 10,
      std::bind(&ObstacleDetector::pointCloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to PointCloud2 topic: %s", lidar_input_topic_.c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Front Obstacle Detector initialized");
  RCLCPP_INFO(this->get_logger(), "ROI: X[%.2f, %.2f] Y[%.2f, %.2f]",
    roi_config_.x_min, roi_config_.x_max,
    roi_config_.y_min, roi_config_.y_max);
}

void ObstacleDetector::loadConfig(const std::string & config_file)
{
  std::ifstream file(config_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open config file: %s", config_file.c_str());
    throw std::runtime_error("Cannot open config file");
  }

  json config;
  file >> config;

  // Load ROI configuration
  roi_config_.x_min = config["roi"]["x_min"].get<double>();
  roi_config_.x_max = config["roi"]["x_max"].get<double>();
  roi_config_.y_min = config["roi"]["y_min"].get<double>();
  roi_config_.y_max = config["roi"]["y_max"].get<double>();

  // Load detection configuration
  detection_config_.min_points_threshold = config["detection"]["min_points_threshold"].get<int>();
  detection_config_.distance_threshold = config["detection"]["distance_threshold"].get<double>();
  detection_config_.publish_rate_hz = config["detection"]["publish_rate_hz"].get<double>();

  // Load topic names
  lidar_input_topic_ = config["topics"]["lidar_input"].get<std::string>();
  distance_output_topic_ = config["topics"]["distance_output"].get<std::string>();
  detected_output_topic_ = config["topics"]["detected_output"].get<std::string>();

  RCLCPP_INFO(this->get_logger(), "Configuration loaded from: %s", config_file.c_str());
}

bool ObstacleDetector::isPointInROI(double x, double y) const
{
  return (x >= roi_config_.x_min && x <= roi_config_.x_max &&
          y >= roi_config_.y_min && y <= roi_config_.y_max);
}

void ObstacleDetector::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::vector<double> distances_in_roi;
  int points_in_roi = 0;

  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    float range = msg->ranges[i];

    // Skip invalid readings
    if (std::isnan(range) || std::isinf(range) ||
        range < msg->range_min || range > msg->range_max) {
      continue;
    }

    // Calculate angle for this reading
    float angle = msg->angle_min + i * msg->angle_increment;

    // Convert polar to cartesian (assuming laser is at origin)
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Check if point is in ROI
    if (isPointInROI(x, y)) {
      distances_in_roi.push_back(range);
      points_in_roi++;
    }
  }

  // Determine if obstacle is detected
  bool detected = points_in_roi >= detection_config_.min_points_threshold;

  // Find minimum distance in ROI
  double min_distance = std::numeric_limits<double>::infinity();
  if (!distances_in_roi.empty()) {
    min_distance = *std::min_element(distances_in_roi.begin(), distances_in_roi.end());
  }

  // Publish results
  publishResults(min_distance, detected);

  // Log detection info
  if (detected) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Obstacle detected! Distance: %.2f m, Points: %d", min_distance, points_in_roi);
  }
}

void ObstacleDetector::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Parse PointCloud2 message
  // Note: This is a simplified implementation. For production, use pcl_conversions

  int point_step = msg->point_step;
  int num_points = msg->width * msg->height;

  std::vector<double> distances_in_roi;
  int points_in_roi = 0;

  for (int i = 0; i < num_points; ++i) {
    size_t offset = i * point_step;

    // Extract XY (assuming standard PointCloud2 format)
    float x, y;
    memcpy(&x, &msg->data[offset + 0], sizeof(float));
    memcpy(&y, &msg->data[offset + 4], sizeof(float));

    // Skip invalid points
    if (std::isnan(x) || std::isnan(y)) {
      continue;
    }

    // Check if point is in ROI (2D only)
    if (isPointInROI(x, y)) {
      double distance = std::sqrt(x*x + y*y);
      distances_in_roi.push_back(distance);
      points_in_roi++;
    }
  }

  // Determine if obstacle is detected
  bool detected = points_in_roi >= detection_config_.min_points_threshold;

  // Find minimum distance in ROI
  double min_distance = std::numeric_limits<double>::infinity();
  if (!distances_in_roi.empty()) {
    min_distance = *std::min_element(distances_in_roi.begin(), distances_in_roi.end());
  }

  // Publish results
  publishResults(min_distance, detected);

  // Log detection info
  if (detected) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Obstacle detected! Distance: %.2f m, Points: %d", min_distance, points_in_roi);
  }
}

void ObstacleDetector::publishResults(double min_distance, bool detected)
{
  // Publish distance
  auto distance_msg = std_msgs::msg::Float32();
  distance_msg.data = static_cast<float>(min_distance);
  distance_pub_->publish(distance_msg);

  // Publish detection status
  auto detected_msg = std_msgs::msg::Bool();
  detected_msg.data = detected;
  detected_pub_->publish(detected_msg);

  // Update state
  last_min_distance_ = min_distance;
  obstacle_detected_ = detected;
}

void ObstacleDetector::publishROIMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->now();
  marker.ns = "roi";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Position and orientation (identity)
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Line width
  marker.scale.x = 0.02;  // Line thickness (2cm)

  // Color (green or red based on detection)
  if (obstacle_detected_) {
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  } else {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  marker.color.a = 1.0;  // Fully opaque

  // Define 4 corner points of ROI rectangle (at ground level, z=0)
  geometry_msgs::msg::Point p1, p2, p3, p4, p5;

  // Bottom-left (rear-left)
  p1.x = roi_config_.x_min;
  p1.y = roi_config_.y_min;
  p1.z = 0.0;

  // Bottom-right (rear-right)
  p2.x = roi_config_.x_max;
  p2.y = roi_config_.y_min;
  p2.z = 0.0;

  // Top-right (front-right)
  p3.x = roi_config_.x_max;
  p3.y = roi_config_.y_max;
  p3.z = 0.0;

  // Top-left (front-left)
  p4.x = roi_config_.x_min;
  p4.y = roi_config_.y_max;
  p4.z = 0.0;

  // Close the rectangle (back to start)
  p5 = p1;

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p3);
  marker.points.push_back(p4);
  marker.points.push_back(p5);

  marker.lifetime = rclcpp::Duration::from_seconds(2.0);

  marker_pub_->publish(marker);
}

void ObstacleDetector::timerCallback()
{
  publishROIMarker();
}

}  // namespace front_obstacle_detect
