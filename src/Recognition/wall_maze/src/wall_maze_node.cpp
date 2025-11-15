#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

class WallMazeNode : public rclcpp::Node
{
public:
    WallMazeNode() : Node("wall_maze_node")
    {
        // Parameters
        this->declare_parameter("scan_range", 3.0);  // 3m scan range
        
        // ROI common parameters
        this->declare_parameter("roi_width", 1.0);         // ROI width
        this->declare_parameter("roi_height", 1.0);        // ROI height
        this->declare_parameter("roi_min_distance", 0.1);  // ROI min distance
        this->declare_parameter("roi_max_distance", 3.0);  // ROI max distance
        
        // Left ROI position
        this->declare_parameter("left_roi_x", 1.0);        // Left ROI center X
        this->declare_parameter("left_roi_y", 1.0);        // Left ROI center Y
        
        // Right ROI position
        this->declare_parameter("right_roi_x", 1.0);       // Right ROI center X
        this->declare_parameter("right_roi_y", -1.0);      // Right ROI center Y
        
        
        scan_range_ = this->get_parameter("scan_range").as_double();
        
        // Get ROI parameters
        roi_width_ = this->get_parameter("roi_width").as_double();
        roi_height_ = this->get_parameter("roi_height").as_double();
        roi_min_distance_ = this->get_parameter("roi_min_distance").as_double();
        roi_max_distance_ = this->get_parameter("roi_max_distance").as_double();
        
        // Get left ROI position
        left_roi_x_ = this->get_parameter("left_roi_x").as_double();
        left_roi_y_ = this->get_parameter("left_roi_y").as_double();
        
        // Get right ROI position
        right_roi_x_ = this->get_parameter("right_roi_x").as_double();
        right_roi_y_ = this->get_parameter("right_roi_y").as_double();
        
        
        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to scan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WallMazeNode::laserCallback, this, std::placeholders::_1));

        // Publishers for left and right walls
        left_wall_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("left_wall_angle", 10);
        left_wall_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("left_wall_distance", 10);
        right_wall_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("right_wall_angle", 10);
        right_wall_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("right_wall_distance", 10);
        
        // Publisher for ROI visualization
        roi_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("wall_maze_roi_marker", 10);
        
        // Publisher for line visualization
        line_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("wall_maze_line_marker", 10);
        
        // Publisher for wall cross track error
        wall_xte_pub_ = this->create_publisher<std_msgs::msg::Float32>("wall_xte", 10);
        
        // Initialize wall distance tracking
        left_wall_distance_ = 0.0;
        right_wall_distance_ = 0.0;
        left_wall_valid_ = false;
        right_wall_valid_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Wall Maze Node initialized");
    }

private:
    struct Point2D {
        double x, y;
    };
    
    struct LineParameters {
        bool valid;
        double rho;    // Distance in meters
        double theta;  // Angle in radians
        Point2D start; // Line start point for visualization
        Point2D end;   // Line end point for visualization
    };

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wall_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wall_distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wall_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wall_distance_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr roi_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wall_xte_pub_;
    
    // Store latest detected lines
    LineParameters left_line_;
    LineParameters right_line_;
    
    // Store latest wall distances
    double left_wall_distance_;
    double right_wall_distance_;
    bool left_wall_valid_;
    bool right_wall_valid_;
    
    // Parameters
    double scan_range_;  // Keep for potential future use
    
    // ROI parameters
    double roi_width_;
    double roi_height_;
    double roi_min_distance_;
    double roi_max_distance_;
    double left_roi_x_;
    double left_roi_y_;
    double right_roi_x_;
    double right_roi_y_;
    
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Get transform from scan frame to base_link
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform(
                "base_link", msg->header.frame_id, 
                msg->header.stamp, std::chrono::milliseconds(100));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to base_link: %s", 
                msg->header.frame_id.c_str(), ex.what());
            return;
        }

        // Process scan data to detect walls with dual ROI filtering
        std::vector<Point2D> left_roi_points;
        std::vector<Point2D> right_roi_points;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] > roi_min_distance_ && std::isfinite(msg->ranges[i]) && msg->ranges[i] <= roi_max_distance_)
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                
                // Point in scan frame
                geometry_msgs::msg::PointStamped scan_point;
                scan_point.header = msg->header;
                scan_point.point.x = msg->ranges[i] * cos(angle);
                scan_point.point.y = msg->ranges[i] * sin(angle);
                scan_point.point.z = 0.0;
                
                // Transform to base_link frame
                geometry_msgs::msg::PointStamped base_point;
                tf2::doTransform(scan_point, base_point, transform_stamped);
                
                Point2D point;
                point.x = base_point.point.x;
                point.y = base_point.point.y;
                
                // Check if point is within LEFT ROI
                if (isPointInROI(point, left_roi_x_, left_roi_y_, roi_width_, roi_height_))
                {
                    left_roi_points.push_back(point);
                }
                
                // Check if point is within RIGHT ROI
                if (isPointInROI(point, right_roi_x_, right_roi_y_, roi_width_, roi_height_))
                {
                    right_roi_points.push_back(point);
                }
            }
        }
        
        // Analyze wall data separately for each ROI
        analyzeWalls(left_roi_points, right_roi_points);
        
        // Publish ROI visualization
        publishROIVisualization(msg->header);
        
        // Publish line visualization
        publishLineVisualization(msg->header);
    }
    
    bool isPointInROI(const Point2D& point, double roi_center_x, double roi_center_y, double width, double height)
    {
        double half_width = width / 2.0;
        double half_height = height / 2.0;
        
        return (point.x >= roi_center_x - half_width && point.x <= roi_center_x + half_width &&
                point.y >= roi_center_y - half_height && point.y <= roi_center_y + half_height);
    }
    
    double calculateDistanceToLine(const Point2D& start, const Point2D& end)
    {
        // Calculate distance from origin (0,0) to line defined by two points
        // Using formula: |ax + by + c| / sqrt(a² + b²)
        // where line equation is ax + by + c = 0
        
        // Convert two points to line equation ax + by + c = 0
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        
        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
            // Degenerate case: start and end points are the same
            return std::sqrt(start.x * start.x + start.y * start.y);
        }
        
        // Line equation: (y2-y1)x - (x2-x1)y + (x2-x1)y1 - (y2-y1)x1 = 0
        // So: a = dy, b = -dx, c = dx*start.y - dy*start.x
        double a = dy;
        double b = -dx;
        double c = dx * start.y - dy * start.x;
        
        // Distance from origin (0,0) to line ax + by + c = 0
        double distance = std::abs(c) / std::sqrt(a * a + b * b);
        
        return distance;
    }
    
    double calculateSignedDistanceToLine(const Point2D& start, const Point2D& end)
    {
        // Calculate signed distance from origin (0,0) to line
        // Positive if origin is on the left side of the line (when looking from start to end)
        // Negative if origin is on the right side of the line
        
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        
        if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) {
            return std::sqrt(start.x * start.x + start.y * start.y);
        }
        
        // Line equation: ax + by + c = 0
        double a = dy;
        double b = -dx;
        double c = dx * start.y - dy * start.x;
        
        // Signed distance
        double signed_distance = c / std::sqrt(a * a + b * b);
        
        return signed_distance;
    }
    
    void publishROIVisualization(const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Helper lambda to create ROI box
        auto createROIBox = [&](int id, double center_x, double center_y, 
                                double width, double height, 
                                double r, double g, double b) {
            // Create ROI box outline
            visualization_msgs::msg::Marker roi_box;
            roi_box.header = header;
            roi_box.header.frame_id = "base_link";
            roi_box.ns = "roi_box";
            roi_box.id = id * 2;  // Even IDs for outlines
            roi_box.type = visualization_msgs::msg::Marker::LINE_STRIP;
            roi_box.action = visualization_msgs::msg::Marker::ADD;
            roi_box.scale.x = 0.02;  // Line width
            roi_box.color.r = r;
            roi_box.color.g = g;
            roi_box.color.b = b;
            roi_box.color.a = 0.8;
            roi_box.lifetime = rclcpp::Duration::from_seconds(0.1);
            
            // Calculate box corners
            double half_width = width / 2.0;
            double half_height = height / 2.0;
            
            geometry_msgs::msg::Point p1, p2, p3, p4, p5;
            // Bottom-left corner
            p1.x = center_x - half_width;
            p1.y = center_y - half_height;
            p1.z = 0.0;
            // Bottom-right corner
            p2.x = center_x + half_width;
            p2.y = center_y - half_height;
            p2.z = 0.0;
            // Top-right corner
            p3.x = center_x + half_width;
            p3.y = center_y + half_height;
            p3.z = 0.0;
            // Top-left corner
            p4.x = center_x - half_width;
            p4.y = center_y + half_height;
            p4.z = 0.0;
            // Close the box
            p5 = p1;
            
            roi_box.points.push_back(p1);
            roi_box.points.push_back(p2);
            roi_box.points.push_back(p3);
            roi_box.points.push_back(p4);
            roi_box.points.push_back(p5);
            
            marker_array.markers.push_back(roi_box);
            
            // Create filled box
            visualization_msgs::msg::Marker roi_filled;
            roi_filled.header = header;
            roi_filled.header.frame_id = "base_link";
            roi_filled.ns = "roi_filled";
            roi_filled.id = id * 2 + 1;  // Odd IDs for filled boxes
            roi_filled.type = visualization_msgs::msg::Marker::CUBE;
            roi_filled.action = visualization_msgs::msg::Marker::ADD;
            roi_filled.pose.position.x = center_x;
            roi_filled.pose.position.y = center_y;
            roi_filled.pose.position.z = 0.0;
            roi_filled.pose.orientation.w = 1.0;
            roi_filled.scale.x = width;
            roi_filled.scale.y = height;
            roi_filled.scale.z = 0.01;
            roi_filled.color.r = r;
            roi_filled.color.g = g;
            roi_filled.color.b = b;
            roi_filled.color.a = 0.1;
            roi_filled.lifetime = rclcpp::Duration::from_seconds(0.1);
            
            marker_array.markers.push_back(roi_filled);
        };
        
        // Create LEFT ROI (blue)
        createROIBox(0, left_roi_x_, left_roi_y_, roi_width_, roi_height_, 0.0, 0.0, 1.0);
        
        // Create RIGHT ROI (yellow)
        createROIBox(1, right_roi_x_, right_roi_y_, roi_width_, roi_height_, 1.0, 1.0, 0.0);
        
        // Publish the marker array
        roi_marker_pub_->publish(marker_array);
    }
    
    void publishLineVisualization(const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        // Visualize left line (blue)
        if (left_line_.valid) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header = header;
            line_marker.header.frame_id = "base_link";
            line_marker.ns = "detected_lines";
            line_marker.id = 0;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.05;  // Line width 5cm
            
            // Blue color for left line
            line_marker.color.r = 0.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 1.0;
            line_marker.color.a = 1.0;
            
            line_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            
            // Add line endpoints
            geometry_msgs::msg::Point p1, p2;
            p1.x = left_line_.start.x;
            p1.y = left_line_.start.y;
            p1.z = 0.0;
            p2.x = left_line_.end.x;
            p2.y = left_line_.end.y;
            p2.z = 0.0;
            
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            
            marker_array.markers.push_back(line_marker);
        }
        
        // Visualize right line (yellow)
        if (right_line_.valid) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header = header;
            line_marker.header.frame_id = "base_link";
            line_marker.ns = "detected_lines";
            line_marker.id = 1;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            line_marker.scale.x = 0.05;  // Line width 5cm
            
            // Yellow color for right line
            line_marker.color.r = 1.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;
            
            line_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
            
            // Add line endpoints
            geometry_msgs::msg::Point p1, p2;
            p1.x = right_line_.start.x;
            p1.y = right_line_.start.y;
            p1.z = 0.0;
            p2.x = right_line_.end.x;
            p2.y = right_line_.end.y;
            p2.z = 0.0;
            
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            
            marker_array.markers.push_back(line_marker);
        }
        
        // Clear old markers if no lines detected
        if (!left_line_.valid && !right_line_.valid) {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header = header;
            delete_marker.header.frame_id = "base_link";
            delete_marker.ns = "detected_lines";
            delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(delete_marker);
        }
        
        // Publish the line markers
        line_marker_pub_->publish(marker_array);
    }
    
    void analyzeWalls(const std::vector<Point2D>& left_points, const std::vector<Point2D>& right_points)
    {
        // Analyze left ROI
        if (!left_points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Left ROI: %zu points", left_points.size());
            left_line_ = detectLineInROI(left_points, "left");
        } else {
            left_line_.valid = false;
            left_wall_valid_ = false;
        }
        
        // Analyze right ROI
        if (!right_points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Right ROI: %zu points", right_points.size());
            right_line_ = detectLineInROI(right_points, "right");
        } else {
            right_line_.valid = false;
            right_wall_valid_ = false;
        }
        
        // Calculate and publish wall cross track error
        publishWallXTE();
    }
    
    void publishWallXTE()
    {
        if (left_wall_valid_ && right_wall_valid_) {
            // Calculate wall XTE: positive means robot is closer to right wall
            double wall_xte = left_wall_distance_ - right_wall_distance_;
            
            std_msgs::msg::Float32 xte_msg;
            xte_msg.data = wall_xte;
            wall_xte_pub_->publish(xte_msg);
            
            RCLCPP_INFO(this->get_logger(), "=== WALL XTE: %.3fm (L:%.3fm - R:%.3fm) ===", 
                        wall_xte, left_wall_distance_, right_wall_distance_);
            
            if (wall_xte > 0.1) {
                RCLCPP_INFO(this->get_logger(), "Robot is CLOSER to RIGHT wall by %.3fm", wall_xte);
            } else if (wall_xte < -0.1) {
                RCLCPP_INFO(this->get_logger(), "Robot is CLOSER to LEFT wall by %.3fm", -wall_xte);
            } else {
                RCLCPP_INFO(this->get_logger(), "Robot is CENTERED between walls (±%.3fm)", std::abs(wall_xte));
            }
        } else if (left_wall_valid_) {
            RCLCPP_WARN(this->get_logger(), "Only LEFT wall detected (%.3fm) - cannot calculate XTE", left_wall_distance_);
        } else if (right_wall_valid_) {
            RCLCPP_WARN(this->get_logger(), "Only RIGHT wall detected (%.3fm) - cannot calculate XTE", right_wall_distance_);
        } else {
            RCLCPP_WARN(this->get_logger(), "No walls detected - cannot calculate XTE");
        }
    }
    
    LineParameters detectLineInROI(const std::vector<Point2D>& points, const std::string& side)
    {
        LineParameters line_params;
        line_params.valid = false;
        
        if (points.size() < 2) {
            return line_params;
        }
        
        // Convert points to OpenCV format for Hough Transform
        // Create a binary image where points are marked
        int img_width = 400;   // 4m width with 1cm resolution
        int img_height = 400;  // 4m height with 1cm resolution
        double resolution = 0.01; // 1cm per pixel
        cv::Mat binary_img = cv::Mat::zeros(img_height, img_width, CV_8UC1);
        
        // Mark points in the binary image
        for (const auto& point : points) {
            // Convert from meters to pixels (center of image is origin)
            int px = static_cast<int>((point.x / resolution) + img_width/2);
            int py = static_cast<int>((img_height/2) - (point.y / resolution));
            
            if (px >= 0 && px < img_width && py >= 0 && py < img_height) {
                // Mark a small region around the point for better line detection
                cv::circle(binary_img, cv::Point(px, py), 2, cv::Scalar(255), -1);
            }
        }
        
        // Apply Hough Line Transform
        std::vector<cv::Vec2f> lines;
        double rho_resolution = 1;  // Distance resolution in pixels
        double theta_resolution = CV_PI / 180;  // Angle resolution in radians (1 degree)
        int threshold = std::max(10, static_cast<int>(points.size() * 0.3));  // At least 30% of points
        
        cv::HoughLines(binary_img, lines, rho_resolution, theta_resolution, threshold);
        
        if (lines.empty()) {
            RCLCPP_WARN(this->get_logger(), "No lines detected in %s ROI", side.c_str());
            return line_params;
        }
        
        // Use the first (strongest) line
        float rho_pixels = lines[0][0];    // Distance from image center in pixels
        float theta = lines[0][1];         // Angle in radians
        
        // Convert rho from pixels to meters (accounting for image center offset)
        // Image center is at (img_width/2, img_height/2) in pixels
        // Robot origin is at image center
        double rho_meters = (rho_pixels - img_width/2) * resolution;
        
        // Convert angle to degrees
        double angle_degrees = theta * 180.0 / CV_PI;
        
        // Normalize angle to [-180, 180]
        while (angle_degrees > 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        
        RCLCPP_INFO(this->get_logger(), "%s ROI Hough: rho_pixels=%.1f, theta=%.2f rad (%.1f deg)", 
                    side.c_str(), rho_pixels, theta, angle_degrees);
        
        // Store line parameters for visualization
        line_params.valid = true;
        line_params.rho = rho_meters;
        line_params.theta = theta;
        
        // Calculate line endpoints in robot coordinate system
        // Convert from Hough space to Cartesian coordinates
        double a = cos(theta);
        double b = sin(theta);
        double x0 = a * rho_pixels;
        double y0 = b * rho_pixels;
        
        // Create a long line segment (extend 1000 pixels in each direction)
        int line_length = 1000;
        double x1_pixels = x0 + line_length * (-b);
        double y1_pixels = y0 + line_length * (a);
        double x2_pixels = x0 - line_length * (-b);
        double y2_pixels = y0 - line_length * (a);
        
        // Convert from pixels to meters in robot frame
        // Remember: image center (img_width/2, img_height/2) corresponds to robot origin (0,0)
        line_params.start.x = (x1_pixels - img_width/2) * resolution;
        line_params.start.y = -(y1_pixels - img_height/2) * resolution;  // Negative because image Y is inverted
        line_params.end.x = (x2_pixels - img_width/2) * resolution;
        line_params.end.y = -(y2_pixels - img_height/2) * resolution;
        
        // Clip the line to reasonable bounds
        double max_coord = 5.0;  // Maximum coordinate in meters
        line_params.start.x = std::max(-max_coord, std::min(max_coord, line_params.start.x));
        line_params.start.y = std::max(-max_coord, std::min(max_coord, line_params.start.y));
        line_params.end.x = std::max(-max_coord, std::min(max_coord, line_params.end.x));
        line_params.end.y = std::max(-max_coord, std::min(max_coord, line_params.end.y));
        
        // Calculate distance from base_link (origin) to the detected line
        double distance_to_line = calculateDistanceToLine(line_params.start, line_params.end);
        double signed_distance = calculateSignedDistanceToLine(line_params.start, line_params.end);
        
        RCLCPP_INFO(this->get_logger(), "%s line: start(%.2f, %.2f) -> end(%.2f, %.2f)",
                    side.c_str(), line_params.start.x, line_params.start.y,
                    line_params.end.x, line_params.end.y);
        
        RCLCPP_INFO(this->get_logger(), "*** %s WALL DISTANCE from base_link: %.3fm (signed: %.3fm) ***",
                    side.c_str(), distance_to_line, signed_distance);
        
        // Publish wall data with calculated distance
        if (side == "left") {
            std_msgs::msg::Float32 angle_msg, distance_msg;
            angle_msg.data = angle_degrees;
            distance_msg.data = distance_to_line;  // Use actual distance from base_link
            left_wall_angle_pub_->publish(angle_msg);
            left_wall_distance_pub_->publish(distance_msg);
            
            // Store left wall distance
            left_wall_distance_ = distance_to_line;
            left_wall_valid_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Left wall: angle=%.2f°, Hough_rho=%.3fm, base_link_distance=%.3fm, points=%zu", 
                        angle_degrees, rho_meters, distance_to_line, points.size());
        } else if (side == "right") {
            std_msgs::msg::Float32 angle_msg, distance_msg;
            angle_msg.data = angle_degrees;
            distance_msg.data = distance_to_line;  // Use actual distance from base_link
            right_wall_angle_pub_->publish(angle_msg);
            right_wall_distance_pub_->publish(distance_msg);
            
            // Store right wall distance
            right_wall_distance_ = distance_to_line;
            right_wall_valid_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Right wall: angle=%.2f°, Hough_rho=%.3fm, base_link_distance=%.3fm, points=%zu", 
                        angle_degrees, rho_meters, distance_to_line, points.size());
        }
        
        // Optional: Detect multiple lines if needed
        if (lines.size() > 1) {
            RCLCPP_DEBUG(this->get_logger(), "Detected %zu lines in %s ROI", lines.size(), side.c_str());
        }
        
        return line_params;
    }
    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallMazeNode>());
    rclcpp::shutdown();
    return 0;
}