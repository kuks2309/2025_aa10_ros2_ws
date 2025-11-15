#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>

class ObstacleVisualizer : public rclcpp::Node
{
public:
    ObstacleVisualizer() : Node("obstacle_visualizer")
    {
        // 파라미터
        this->declare_parameter("max_range", 10.0);
        this->declare_parameter("angle_resolution", 1.0);  // degrees
        
        max_range_ = this->get_parameter("max_range").as_double();
        angle_resolution_ = this->get_parameter("angle_resolution").as_double() * M_PI / 180.0;
        
        // TF2 초기화
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 구독자 - 지도
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos, std::bind(&ObstacleVisualizer::mapCallback, this, std::placeholders::_1));
        
        // 발행자 - 장애물 마커
        obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle_points", 10);
        
        // 타이머 (10Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObstacleVisualizer::visualizeObstacles, this));
        
        RCLCPP_INFO(this->get_logger(), "Obstacle Visualizer started!");
        RCLCPP_INFO(this->get_logger(), "  Max range: %.1fm", max_range_);
        RCLCPP_INFO(this->get_logger(), "  Showing all detected obstacles in BLUE");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = msg;
        map_received_ = true;
    }
    
    void visualizeObstacles()
    {
        if (!map_received_) {
            return;
        }
        
        // 로봇 위치 가져오기 (map 좌표계)
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "map", "base_link", this->now(), tf2::durationFromSec(0.5));
        } catch (tf2::TransformException &ex) {
            RCLCPP_DEBUG(this->get_logger(), "TF failed: %s", ex.what());
            return;
        }
        
        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        
        // 로봇 방향
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 장애물 점들 수집
        std::vector<geometry_msgs::msg::Point> obstacle_points;
        
        // 360도 전체 스캔
        for (double angle = -M_PI; angle < M_PI; angle += angle_resolution_) {
            // 맵 좌표계에서의 각도
            double map_angle = yaw + angle;
            double dx = cos(map_angle);
            double dy = sin(map_angle);
            
            // 레이를 따라 스캔
            for (double dist = 0.1; dist <= max_range_; dist += 0.05) {
                double x = robot_x + dist * dx;
                double y = robot_y + dist * dy;
                
                // 맵 인덱스로 변환
                int mx = (x - current_map_->info.origin.position.x) / current_map_->info.resolution;
                int my = (y - current_map_->info.origin.position.y) / current_map_->info.resolution;
                
                // 범위 확인
                if (mx < 0 || mx >= (int)current_map_->info.width ||
                    my < 0 || my >= (int)current_map_->info.height) {
                    break;
                }
                
                // 장애물 확인 (>50 = 장애물)
                int index = my * current_map_->info.width + mx;
                if (current_map_->data[index] > 50) {
                    geometry_msgs::msg::Point p;
                    p.x = x;
                    p.y = y;
                    p.z = 0.0;
                    obstacle_points.push_back(p);
                    break;  // 첫 번째 장애물만 기록
                }
            }
        }
        
        // 마커 발행
        publishObstacleMarker(obstacle_points);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Visualizing %zu obstacle points", obstacle_points.size());
    }
    
    void publishObstacleMarker(const std::vector<geometry_msgs::msg::Point>& points)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "obstacles";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 파란색 점들 (크기 확대)
        marker.scale.x = 0.3;  // 점 크기
        marker.scale.y = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // 파란색
        marker.color.a = 1.0;
        
        marker.points = points;
        
        obstacle_marker_pub_->publish(marker);
    }
    
    // 멤버 변수
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    bool map_received_ = false;
    
    double max_range_;
    double angle_resolution_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleVisualizer>());
    rclcpp::shutdown();
    return 0;
}