#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

// Control mode definitions (from aa10_car_yaw_control)
#define IMU_CONTROL       0
#define LANE_CONTROL      1
#define MAZE_CONTROL      2
#define STEER_CONTROL     3
#define LIDAR_CONTROL     4

class ConeNavigationNode : public rclcpp::Node
{
public:
    ConeNavigationNode() : Node("cone_navigation")
    {
        // Parameters
        this->declare_parameter("inflation_radius", 0.125);  // 12.5cm default
        this->declare_parameter("scan_angle_step", 5.0);  // 5 degrees step
        this->declare_parameter("scan_range", 30.0);  // ±30 degrees total range
        this->declare_parameter("scan_distance", 0.8);  // 0.8m scan distance
        this->declare_parameter("blocked_distance", 0.6);  // Distance threshold for blocked path
        this->declare_parameter("points_per_ray", 20);  // Number of points per scan ray
        
        // Map parameters
        this->declare_parameter("map_length", 1.0);  // Forward distance (X axis)
        this->declare_parameter("map_width", 0.9);    // Lateral width (Y axis)
        this->declare_parameter("map_resolution", 0.05); // Resolution in meters per cell
        
        // ROI parameters
        this->declare_parameter("roi_width", 0.4);   // Lateral ROI width
        this->declare_parameter("roi_forward", 0.7); // Forward ROI distance

        inflation_radius_ = this->get_parameter("inflation_radius").as_double();
        scan_angle_step_ = this->get_parameter("scan_angle_step").as_double();
        scan_range_ = this->get_parameter("scan_range").as_double();
        scan_distance_ = this->get_parameter("scan_distance").as_double();
        blocked_distance_ = this->get_parameter("blocked_distance").as_double();
        points_per_ray_ = this->get_parameter("points_per_ray").as_int();
        
        // Get map parameters
        map_length_ = this->get_parameter("map_length").as_double();
        map_width_ = this->get_parameter("map_width").as_double();
        map_resolution_ = this->get_parameter("map_resolution").as_double();
        
        // Get ROI parameters
        roi_width_ = this->get_parameter("roi_width").as_double();
        roi_forward_ = this->get_parameter("roi_forward").as_double();
        
        // Calculate map dimensions
        map_cols_ = static_cast<int>(map_length_ / map_resolution_);
        map_rows_ = static_cast<int>(map_width_ / map_resolution_);
        
        // Initialize steering control mode
        steering_control_mode_ = -1;
        cone_maze_enable_ = false;
        
        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to scan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ConeNavigationNode::laserCallback, this, std::placeholders::_1));
            
        // Subscribe to steering control mode
        steering_control_mode_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/Car_Control_Cmd/steering_control_mode", 10, std::bind(&ConeNavigationNode::steeringControlModeCallback, this, std::placeholders::_1));

        // Publish obstacle map
        obstacle_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("obstacle_map", 10);

        // Publish inflation map
        inflation_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("inflation_map", 10);

        // Publish best trajectory from Bendy Ruler
        best_trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("best_trajectory", 10);
        trajectory_candidates_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_candidates", 10);
        distance_text_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("distance_text", 10);
        heading_arrow_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("heading_arrow", 10);
        target_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("cone_track_target_angle", 10);
        maze_xte_pub_ = this->create_publisher<std_msgs::msg::Float32>("/xte/maze", 10);

        RCLCPP_INFO(this->get_logger(), "Node initialized");
    }

private:
    // Data structures
    struct Point2D
    {
        double x;
        double y;
    };

    struct Trajectory
    {
        std::vector<Point2D> points;
        double curvature;  // 0 for straight, positive for left, negative for right
        double cost;
        bool collision;
        double free_distance;  // Distance to first obstacle
    };



    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr steering_control_mode_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflation_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_candidates_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr distance_text_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr heading_arrow_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr maze_xte_pub_;

    // Parameters
    double inflation_radius_;
    double scan_angle_step_;
    double scan_range_;
    double scan_distance_;
    double blocked_distance_;
    int points_per_ray_;
    
    // Map parameters
    double map_length_;
    double map_width_;
    double map_resolution_;
    int map_cols_;
    int map_rows_;
    
    // ROI parameters
    double roi_width_;
    double roi_forward_;
    
    // Steering control mode
    int steering_control_mode_;
    bool cone_maze_enable_;

    // Store latest inflation map for Bendy Ruler
    nav_msgs::msg::OccupancyGrid latest_inflation_map_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void steeringControlModeCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        steering_control_mode_ = msg->data;

        // Enable cone maze navigation only for MAZE_CONTROL mode
        bool previous_state = cone_maze_enable_;
        cone_maze_enable_ = (steering_control_mode_ == MAZE_CONTROL);

        RCLCPP_INFO(this->get_logger(), "Steering control mode: %d", steering_control_mode_);

        if (previous_state != cone_maze_enable_) {
            RCLCPP_INFO(this->get_logger(), "Cone maze navigation %s based on control mode",
                       cone_maze_enable_ ? "ENABLED" : "DISABLED");
        }
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!cone_maze_enable_) {
            return;
        }
        // Get transform from scan frame to base_link
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            // Use the timestamp from the laser scan message
            transform_stamped = tf_buffer_->lookupTransform(
                "base_link", msg->header.frame_id, 
                msg->header.stamp, std::chrono::milliseconds(100));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to base_link: %s", 
                msg->header.frame_id.c_str(), ex.what());
            return;
        }

        // Process scan data for ROI
        std::vector<Point2D> roi_points;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] > 0.1 && std::isfinite(msg->ranges[i]))
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
                
                double x = base_point.point.x;
                double y = base_point.point.y;

                // ROI check in base_link frame
                if (x > 0 && x <= roi_forward_ && std::abs(y) <= roi_width_)
                {
                    Point2D point;
                    point.x = x;
                    point.y = y;
                    roi_points.push_back(point);
                }
            }
        }

        // Create and publish obstacle map
        publishObstacleMap(roi_points, msg->header);

        // Create and publish inflation map
        publishInflationMap(roi_points, msg->header);

        // Run Bendy Ruler algorithm on inflation map
        runBendyRuler(msg->header);

    }


    void publishObstacleMap(const std::vector<Point2D>& roi_points, const std_msgs::msg::Header& header)
    {
        nav_msgs::msg::OccupancyGrid obstacle_map;

        // Header setup
        obstacle_map.header = header;
        obstacle_map.header.frame_id = "base_link";

        // Map metadata (width=columns, height=rows)
        obstacle_map.info.width = map_cols_;   // X축 방향 cells
        obstacle_map.info.height = map_rows_;  // Y축 방향 cells
        obstacle_map.info.resolution = map_resolution_;

        // Map origin (bottom-left corner in base_link frame)
        // Map covers: x from 0 to 0.5m (forward), y from -0.8 to +0.8m (lateral)
        obstacle_map.info.origin.position.x = 0.0;
        obstacle_map.info.origin.position.y = -map_width_ / 2.0;
        obstacle_map.info.origin.position.z = 0.0;
        obstacle_map.info.origin.orientation.w = 1.0;

        // Initialize map data (unknown = -1, free = 0, occupied = 100)
        obstacle_map.data.resize(map_cols_ * map_rows_, 0);  // Start with free space

        // Mark obstacle cells based on lidar points (binary: 0=free, 100=occupied)
        for (const auto& point : roi_points)
        {
            // Convert world coordinates to grid coordinates
            int grid_x = static_cast<int>((point.x - obstacle_map.info.origin.position.x) / map_resolution_);
            int grid_y = static_cast<int>((point.y - obstacle_map.info.origin.position.y) / map_resolution_);

            // Check bounds
            if (grid_x >= 0 && grid_x < map_cols_ && grid_y >= 0 && grid_y < map_rows_)
            {
                int index = grid_y * map_cols_ + grid_x;  // row * width + col
                obstacle_map.data[index] = 100;  // Mark as occupied (binary)
            }
        }

        obstacle_map_pub_->publish(obstacle_map);

        RCLCPP_INFO(this->get_logger(), "Obstacle map: %dx%d cells, %.2fm resolution, %zu obstacles",
                   map_cols_, map_rows_, map_resolution_, roi_points.size());
    }

    void publishInflationMap(const std::vector<Point2D>& roi_points, const std_msgs::msg::Header& header)
    {
        nav_msgs::msg::OccupancyGrid inflation_map;

        // Header setup
        inflation_map.header = header;
        inflation_map.header.frame_id = "base_link";

        // Map metadata (same as obstacle map)
        inflation_map.info.width = map_cols_;
        inflation_map.info.height = map_rows_;
        inflation_map.info.resolution = map_resolution_;

        // Map origin (same as obstacle map)
        inflation_map.info.origin.position.x = 0.0;
        inflation_map.info.origin.position.y = -map_width_ / 2.0;
        inflation_map.info.origin.position.z = 0.0;
        inflation_map.info.origin.orientation.w = 1.0;

        // Initialize map data (0 = free, 100 = obstacle, 50 = inflated)
        inflation_map.data.resize(map_cols_ * map_rows_, 0);

        // Calculate inflation cells in grid units
        int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / map_resolution_));

        // First pass: mark original obstacles
        for (const auto& point : roi_points)
        {
            int grid_x = static_cast<int>((point.x - inflation_map.info.origin.position.x) / map_resolution_);
            int grid_y = static_cast<int>((point.y - inflation_map.info.origin.position.y) / map_resolution_);

            if (grid_x >= 0 && grid_x < map_cols_ && grid_y >= 0 && grid_y < map_rows_)
            {
                int index = grid_y * map_cols_ + grid_x;
                inflation_map.data[index] = 100;  // Mark as obstacle
            }
        }

        // Second pass: apply inflation around obstacles
        std::vector<int8_t> inflated_map = inflation_map.data;  // Copy for inflation

        for (const auto& point : roi_points)
        {
            int center_x = static_cast<int>((point.x - inflation_map.info.origin.position.x) / map_resolution_);
            int center_y = static_cast<int>((point.y - inflation_map.info.origin.position.y) / map_resolution_);

            // Apply circular inflation
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
            {
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
                {
                    int inflated_x = center_x + dx;
                    int inflated_y = center_y + dy;

                    // Check if within map bounds
                    if (inflated_x >= 0 && inflated_x < map_cols_ &&
                        inflated_y >= 0 && inflated_y < map_rows_)
                    {
                        // Calculate distance from obstacle center
                        double distance = std::sqrt(dx * dx + dy * dy) * map_resolution_;

                        // Apply inflation if within radius
                        if (distance <= inflation_radius_)
                        {
                            int inflated_index = inflated_y * map_cols_ + inflated_x;

                            // Only inflate if not already an obstacle
                            if (inflated_map[inflated_index] != 100)
                            {
                                inflated_map[inflated_index] = 50;  // Mark as inflated
                            }
                        }
                    }
                }
            }
        }

        inflation_map.data = inflated_map;
        inflation_map_pub_->publish(inflation_map);

        // Store for Bendy Ruler
        latest_inflation_map_ = inflation_map;

        RCLCPP_INFO(this->get_logger(), "Inflation map: radius=%.3fm (%d cells), obstacles=%zu",
                   inflation_radius_, inflation_cells, roi_points.size());
    }

    void runBendyRuler(const std_msgs::msg::Header& header)
    {
        if (latest_inflation_map_.data.empty()) return;

        std::vector<Trajectory> trajectories;

        // Convert degrees to radians
        double angle_step_rad = scan_angle_step_ * M_PI / 180.0;
        double range_rad = scan_range_ * M_PI / 180.0;

        // Calculate number of rays
        int num_rays = static_cast<int>(2 * range_rad / angle_step_rad) + 1;

        // Scan from -range to +range
        for (int i = 0; i < num_rays; ++i)
        {
            Trajectory traj;

            // Calculate angle for this ray
            double angle = -range_rad + i * angle_step_rad;
            traj.curvature = angle;  // Store angle in curvature field

            // Generate straight line in that direction
            for (int j = 0; j <= points_per_ray_; ++j)
            {
                double distance = (j * scan_distance_) / points_per_ray_;
                Point2D point;
                point.x = distance * std::cos(angle);
                point.y = distance * std::sin(angle);
                traj.points.push_back(point);
            }

            // Simple evaluation: check for obstacles
            evaluateTrajectory(traj);
            trajectories.push_back(traj);
        }

        // Find best trajectory (lowest cost)
        int best_idx = -1;
        double min_cost = std::numeric_limits<double>::max();

        // Display all angles and distances
        RCLCPP_INFO(this->get_logger(), "=== Ray Scan Results (blocked < %.1fm) ===", blocked_distance_);
        for (size_t i = 0; i < trajectories.size(); ++i)
        {
            double angle_deg = trajectories[i].curvature * 180.0 / M_PI;
            double distance = trajectories[i].free_distance;

            // Check if path is blocked based on distance threshold
            bool is_blocked = (distance < blocked_distance_);
            trajectories[i].collision = is_blocked;  // Update collision status

            // Format the output
            std::string status = is_blocked ? "BLOCKED" : "CLEAR";
            RCLCPP_INFO(this->get_logger(), "Angle %+6.1f°: %.2fm [%s]",
                       angle_deg, distance, status.c_str());

            // Find best trajectory (not blocked and lowest cost)
            if (!is_blocked && trajectories[i].cost < min_cost)
            {
                min_cost = trajectories[i].cost;
                best_idx = i;
            }
        }

        // Publish visualization
        publishTrajectoryVisualization(trajectories, best_idx, header);

        // Publish heading arrow
        publishHeadingArrow(trajectories, best_idx, header);

        // Publish target angle
        std_msgs::msg::Float32 target_angle_msg;
        if (best_idx >= 0)
        {
            target_angle_msg.data = static_cast<float>(trajectories[best_idx].curvature * 180.0 / M_PI);
        }
        else
        {
            target_angle_msg.data = 0.0f;  // Default to straight if no path found
        }
        target_angle_pub_->publish(target_angle_msg);

        // Log best result
        if (best_idx >= 0)
        {
            double best_angle_deg = trajectories[best_idx].curvature * 180.0 / M_PI;
            RCLCPP_INFO(this->get_logger(), ">>> BEST: %+.1f° (distance: %.2fm, cost: %.1f)",
                       best_angle_deg, trajectories[best_idx].free_distance, trajectories[best_idx].cost);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), ">>> No clear path found!");
        }
        RCLCPP_INFO(this->get_logger(), "=======================");
    }

    void evaluateTrajectory(Trajectory& traj)
    {
        traj.cost = 0;
        traj.collision = false;  // Will be updated later based on blocked_distance
        traj.free_distance = scan_distance_;  // Initialize to max distance

        // Simple evaluation: check points along the ray
        for (size_t i = 0; i < traj.points.size(); ++i)
        {
            const auto& point = traj.points[i];

            // Convert to grid coordinates
            int grid_x = static_cast<int>((point.x - latest_inflation_map_.info.origin.position.x) / map_resolution_);
            int grid_y = static_cast<int>((point.y - latest_inflation_map_.info.origin.position.y) / map_resolution_);

            // Check bounds
            if (grid_x >= 0 && grid_x < map_cols_ && grid_y >= 0 && grid_y < map_rows_)
            {
                int index = grid_y * map_cols_ + grid_x;
                int cell_value = latest_inflation_map_.data[index];

                if (cell_value > 0)  // Any obstacle or inflation
                {
                    // Calculate distance to this point
                    traj.free_distance = std::sqrt(point.x * point.x + point.y * point.y);
                    traj.cost += cell_value;  // Simple cost accumulation
                    break;  // Stop at first obstacle
                }
            }
        }

        // Add small angle penalty to prefer straight
        traj.cost += std::abs(traj.curvature) * 180.0 / M_PI * 0.1;  // 0.1 per degree
    }

    void publishTrajectoryVisualization(const std::vector<Trajectory>& trajectories,
                                       int best_idx, const std_msgs::msg::Header& header)
    {
        // Publish all trajectory candidates
        visualization_msgs::msg::MarkerArray candidates_array;

        // Clear previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.ns = "trajectory_candidates";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        candidates_array.markers.push_back(clear_marker);

        // Add each trajectory
        for (size_t i = 0; i < trajectories.size(); ++i)
        {
            visualization_msgs::msg::Marker traj_marker;
            traj_marker.header = header;
            traj_marker.ns = "trajectory_candidates";
            traj_marker.id = i + 1;
            traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            traj_marker.action = visualization_msgs::msg::Marker::ADD;

            traj_marker.scale.x = 0.005;  // Line width

            // Color based on collision and selection
            if (static_cast<int>(i) == best_idx)
            {
                // Best trajectory - thick green
                traj_marker.scale.x = 0.02;
                traj_marker.color.r = 0.0;
                traj_marker.color.g = 1.0;
                traj_marker.color.b = 0.0;
                traj_marker.color.a = 1.0;
            }
            else if (trajectories[i].collision)
            {
                // Collision - red
                traj_marker.color.r = 1.0;
                traj_marker.color.g = 0.0;
                traj_marker.color.b = 0.0;
                traj_marker.color.a = 0.3;
            }
            else
            {
                // Free - light blue
                traj_marker.color.r = 0.0;
                traj_marker.color.g = 0.5;
                traj_marker.color.b = 1.0;
                traj_marker.color.a = 0.3;
            }

            // Add points up to free distance
            double display_distance = trajectories[i].free_distance;
            for (const auto& point : trajectories[i].points)
            {
                double point_distance = std::sqrt(point.x * point.x + point.y * point.y);
                if (point_distance > display_distance) break;  // Stop at obstacle

                geometry_msgs::msg::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0.01;  // Slightly above ground
                traj_marker.points.push_back(p);
            }

            traj_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            candidates_array.markers.push_back(traj_marker);
        }

        trajectory_candidates_pub_->publish(candidates_array);

        // Publish distance text markers
        visualization_msgs::msg::MarkerArray text_array;

        // Clear previous text
        visualization_msgs::msg::Marker clear_text;
        clear_text.header = header;
        clear_text.ns = "distance_text";
        clear_text.id = 0;
        clear_text.action = visualization_msgs::msg::Marker::DELETEALL;
        text_array.markers.push_back(clear_text);

        // Add distance text for each trajectory
        for (size_t i = 0; i < trajectories.size(); ++i)
        {
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = header;
            text_marker.ns = "distance_text";
            text_marker.id = i + 1;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;

            // Position at end of ray or obstacle point
            size_t end_idx = std::min(static_cast<size_t>(trajectories[i].free_distance / scan_distance_ * trajectories[i].points.size()),
                                     trajectories[i].points.size() - 1);
            if (end_idx < trajectories[i].points.size())
            {
                text_marker.pose.position.x = trajectories[i].points[end_idx].x;
                text_marker.pose.position.y = trajectories[i].points[end_idx].y;
                text_marker.pose.position.z = 0.1;
            }

            text_marker.scale.z = 0.05;  // Text height

            // Format text with angle and distance
            double angle_deg = trajectories[i].curvature * 180.0 / M_PI;
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "%.0f°:%.1fm", angle_deg, trajectories[i].free_distance);
            text_marker.text = buffer;

            // Color based on status
            if (static_cast<int>(i) == best_idx)
            {
                text_marker.color.r = 0.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 0.0;
                text_marker.color.a = 1.0;
            }
            else if (trajectories[i].collision)
            {
                text_marker.color.r = 1.0;
                text_marker.color.g = 0.0;
                text_marker.color.b = 0.0;
                text_marker.color.a = 0.8;
            }
            else
            {
                text_marker.color.r = 0.0;
                text_marker.color.g = 0.5;
                text_marker.color.b = 1.0;
                text_marker.color.a = 0.8;
            }

            text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            text_array.markers.push_back(text_marker);
        }

        distance_text_pub_->publish(text_array);

        // Publish best trajectory separately
        if (best_idx >= 0)
        {
            visualization_msgs::msg::Marker best_marker;
            best_marker.header = header;
            best_marker.ns = "best_trajectory";
            best_marker.id = 1;
            best_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            best_marker.action = visualization_msgs::msg::Marker::ADD;

            best_marker.scale.x = 0.02;  // Thick line
            best_marker.color.r = 0.0;
            best_marker.color.g = 1.0;
            best_marker.color.b = 0.0;
            best_marker.color.a = 1.0;

            // Add points up to free distance
            double best_display_distance = trajectories[best_idx].free_distance;
            for (const auto& point : trajectories[best_idx].points)
            {
                double point_distance = std::sqrt(point.x * point.x + point.y * point.y);
                if (point_distance > best_display_distance) break;  // Stop at obstacle

                geometry_msgs::msg::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0.02;
                best_marker.points.push_back(p);
            }

            best_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            best_trajectory_pub_->publish(best_marker);
        }
    }

    void publishHeadingArrow(const std::vector<Trajectory>& trajectories, int best_idx, const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header = header;
        arrow_marker.ns = "heading_arrow";
        arrow_marker.id = 1;
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;

        // Position at robot center
        arrow_marker.pose.position.x = 0.0;
        arrow_marker.pose.position.y = 0.0;
        arrow_marker.pose.position.z = 0.05;

        if (best_idx >= 0)
        {
            // Calculate heading angle
            double heading_angle = trajectories[best_idx].curvature;  // radians

            // Convert to quaternion (rotation around Z-axis)
            arrow_marker.pose.orientation.x = 0.0;
            arrow_marker.pose.orientation.y = 0.0;
            arrow_marker.pose.orientation.z = std::sin(heading_angle / 2.0);
            arrow_marker.pose.orientation.w = std::cos(heading_angle / 2.0);

            // Arrow size
            arrow_marker.scale.x = 0.3;  // Length
            arrow_marker.scale.y = 0.05; // Width
            arrow_marker.scale.z = 0.05; // Height

            // Green color for clear path
            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 1.0;
            arrow_marker.color.b = 0.0;
            arrow_marker.color.a = 1.0;
        }
        else
        {
            // No clear path - point forward with red color
            arrow_marker.pose.orientation.x = 0.0;
            arrow_marker.pose.orientation.y = 0.0;
            arrow_marker.pose.orientation.z = 0.0;
            arrow_marker.pose.orientation.w = 1.0;

            // Arrow size
            arrow_marker.scale.x = 0.3;  // Length
            arrow_marker.scale.y = 0.05; // Width
            arrow_marker.scale.z = 0.05; // Height

            // Red color for blocked
            arrow_marker.color.r = 1.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 0.0;
            arrow_marker.color.a = 1.0;
        }

        arrow_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        heading_arrow_pub_->publish(arrow_marker);

        // Also publish angle text
        visualization_msgs::msg::Marker text_marker;
        text_marker.header = header;
        text_marker.ns = "heading_text";
        text_marker.id = 1;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        // Position above the arrow
        text_marker.pose.position.x = 0.0;
        text_marker.pose.position.y = 0.0;
        text_marker.pose.position.z = 0.2;

        text_marker.scale.z = 0.08;  // Text height

        if (best_idx >= 0)
        {
            double angle_deg = trajectories[best_idx].curvature * 180.0 / M_PI;
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "Heading: %+.1f°\nDist: %.2fm",
                    angle_deg, trajectories[best_idx].free_distance);
            text_marker.text = buffer;

            // Green color
            text_marker.color.r = 0.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 0.0;
            text_marker.color.a = 1.0;
        }
        else
        {
            text_marker.text = "NO CLEAR PATH";

            // Red color
            text_marker.color.r = 1.0;
            text_marker.color.g = 0.0;
            text_marker.color.b = 0.0;
            text_marker.color.a = 1.0;
        }

        text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        heading_arrow_pub_->publish(text_marker);
    }

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeNavigationNode>());
    rclcpp::shutdown();
    return 0;
}