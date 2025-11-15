#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <opencv2/opencv.hpp>

// 1. laser scan 데이터 저장 구조체
struct ScanPoint
{
    double x, y;
    double angle;
    double range;
    ScanPoint(double x_, double y_, double a_, double r_) : x(x_), y(y_), angle(a_), range(r_) {}
};

struct HoughLine
{
    double rho;
    double theta;
    int votes;
    HoughLine(double r, double t, int v) : rho(r), theta(t), votes(v) {}
};

class MapWallDetectorNode : public rclcpp::Node
{
public:
    MapWallDetectorNode()
        : Node("map_wall_detector")
        , tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
        , tf_listener_(tf_buffer_)
    {
        // QoS 설정
        auto qos = rclcpp::QoS(10);
        qos.transient_local().reliable();

        // 파라미터 선언
        this->declare_parameter("analysis_range_x", 5.0);
        this->declare_parameter("analysis_range_y", 5.0);
        this->declare_parameter("use_opencv_hough", false);
        this->declare_parameter("opencv_rho_resolution", 1.0);
        this->declare_parameter("opencv_theta_resolution", 1.0);
        this->declare_parameter("opencv_threshold", 50);
        this->declare_parameter("image_size", 400);

        // 구독자 생성
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos, std::bind(&MapWallDetectorNode::mapCallback, this, std::placeholders::_1));
        
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MapWallDetectorNode::laserCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Map Wall Detector Node initialized");
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = msg;
        
        // Map 데이터로 Hough 변환 수행
        std::vector<ScanPoint> map_points = extractOccupiedCells();
        if (!map_points.empty()) {
            std::vector<HoughLine> map_lines;
            
            bool use_opencv = this->get_parameter("use_opencv_hough").as_bool();
            if (use_opencv) {
                performOpenCVHoughTransform(map_points, map_lines);
            } else {
                performHoughTransform(map_points, map_lines);
            }
            
            // Map 기반 결과 저장 (scan과 비교용)
            map_hough_lines_ = map_lines;
            
            displayResults(map_lines, "Map");
        }
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 1단계: laser scan 데이터 저장
        std::vector<ScanPoint> scan_data;
        storeLaserScanData(msg, scan_data);
        
        // 2단계: map tf로 변환
        std::vector<ScanPoint> map_data;
        if (!convertScanToMap(msg, scan_data, map_data))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to convert scan to map coordinates");
            return;
        }
        
        // 3단계: hough 변환 실시
        std::vector<HoughLine> detected_lines;
        
        bool use_opencv = this->get_parameter("use_opencv_hough").as_bool();
        if (use_opencv) {
            performOpenCVHoughTransform(map_data, detected_lines);
        } else {
            performHoughTransform(map_data, detected_lines);
        }
        
        // 결과 출력 - Scan과 Map 비교
        displayResults(detected_lines, "Scan");
        
        // Map 기반 결과도 함께 출력 (비교용)
        if (!map_hough_lines_.empty()) {
            displayResults(map_hough_lines_, "Map");
        }
    }
    
    // 1. laser scan 데이터 저장 함수
    void storeLaserScanData(const sensor_msgs::msg::LaserScan::SharedPtr msg, std::vector<ScanPoint>& scan_data)
    {
        scan_data.clear();
        
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float range = msg->ranges[i];
            if (range > msg->range_min && range < msg->range_max && range < 10.0)
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                double x = range * cos(angle);
                double y = range * sin(angle);
                
                scan_data.emplace_back(x, y, angle, range);
            }
        }
        
    }
    
    // 2. laser scan에서 map tf로 데이터 변환 함수
    bool convertScanToMap(const sensor_msgs::msg::LaserScan::SharedPtr msg, 
                         const std::vector<ScanPoint>& scan_data, 
                         std::vector<ScanPoint>& map_data)
    {
        map_data.clear();
        
        try
        {
            // laser_link에서 map으로의 변환
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
            
            for (const auto& point : scan_data)
            {
                // laser 좌표를 stamped point로 변환
                geometry_msgs::msg::PointStamped laser_point;
                laser_point.header = msg->header;
                laser_point.point.x = point.x;
                laser_point.point.y = point.y;
                laser_point.point.z = 0.0;
                
                // map 좌표로 변환
                geometry_msgs::msg::PointStamped map_point;
                tf2::doTransform(laser_point, map_point, transform);
                
                map_data.emplace_back(map_point.point.x, map_point.point.y, point.angle, point.range);
            }
            
            return true;
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
            return false;
        }
    }
    
    // Map에서 occupied cell들을 추출하는 함수
    std::vector<ScanPoint> extractOccupiedCells()
    {
        std::vector<ScanPoint> map_points;
        
        if (!current_map_) {
            RCLCPP_WARN(this->get_logger(), "No map data available");
            return map_points;
        }
        
        // 로봇 위치 가져오기
        try {
            geometry_msgs::msg::TransformStamped robot_transform = tf_buffer_.lookupTransform(
                "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
            
            double robot_x = robot_transform.transform.translation.x;
            double robot_y = robot_transform.transform.translation.y;
            
            double range_x = this->get_parameter("analysis_range_x").as_double();
            double range_y = this->get_parameter("analysis_range_y").as_double();
            double half_x = range_x / 2.0;
            double half_y = range_y / 2.0;
            
            // Map 정보
            double resolution = current_map_->info.resolution;
            double origin_x = current_map_->info.origin.position.x;
            double origin_y = current_map_->info.origin.position.y;
            int width = current_map_->info.width;
            int height = current_map_->info.height;
            
            // 관심 영역의 픽셀 범위 계산
            int min_px = std::max(0, (int)((robot_x - half_x - origin_x) / resolution));
            int max_px = std::min(width - 1, (int)((robot_x + half_x - origin_x) / resolution));
            int min_py = std::max(0, (int)((robot_y - half_y - origin_y) / resolution));
            int max_py = std::min(height - 1, (int)((robot_y + half_y - origin_y) / resolution));
            
            // Occupied cell들 추출 (값이 > 50인 셀들)
            for (int py = min_py; py <= max_py; py++) {
                for (int px = min_px; px <= max_px; px++) {
                    int index = py * width + px;
                    if (index >= 0 && index < (int)current_map_->data.size()) {
                        int8_t cell_value = current_map_->data[index];
                        if (cell_value > 50) {  // Occupied cell
                            // 픽셀 좌표를 실제 좌표로 변환
                            double x = px * resolution + origin_x;
                            double y = py * resolution + origin_y;
                            
                            // 로봇 중심으로부터의 거리 확인
                            double dx = x - robot_x;
                            double dy = y - robot_y;
                            if (std::abs(dx) <= half_x && std::abs(dy) <= half_y) {
                                // angle과 range는 임시값 (map에서는 의미 없음)
                                map_points.emplace_back(x, y, 0.0, sqrt(dx*dx + dy*dy));
                            }
                        }
                    }
                }
            }
        }
        catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Robot position failed for map extraction: %s", ex.what());
            return map_points;
        }
        
        return map_points;
    }
    
    // 3-1. OpenCV Hough 변환 실시
    void performOpenCVHoughTransform(const std::vector<ScanPoint>& map_data, std::vector<HoughLine>& lines)
    {
        if (map_data.empty()) return;
        
        // 범위 필터링 (로봇 중심 5x5미터)
        std::vector<ScanPoint> filtered_data = filterByRange(map_data);
        
        if (filtered_data.size() < 10) return;
        
        // 파라미터 가져오기
        int image_size = this->get_parameter("image_size").as_int();
        double rho_resolution = this->get_parameter("opencv_rho_resolution").as_double();
        double theta_resolution = this->get_parameter("opencv_theta_resolution").as_double() * M_PI / 180.0;
        int threshold = this->get_parameter("opencv_threshold").as_int();
        
        // 좌표 범위 계산
        double min_x = filtered_data[0].x, max_x = filtered_data[0].x;
        double min_y = filtered_data[0].y, max_y = filtered_data[0].y;
        
        for (const auto& point : filtered_data) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        
        // 이미지 생성 (흑백)
        cv::Mat image = cv::Mat::zeros(image_size, image_size, CV_8UC1);
        
        // 좌표 변환 및 점 그리기
        double scale_x = (image_size - 1) / (max_x - min_x);
        double scale_y = (image_size - 1) / (max_y - min_y);
        double scale = std::min(scale_x, scale_y);
        
        for (const auto& point : filtered_data) {
            // 올바른 좌표 변환: x → -v, y → u
            int px = (int)((point.y - min_y) * scale);     // u = y
            int py = (int)((-point.x - (-max_x)) * scale); // v = -x
            
            if (px >= 0 && px < image_size && py >= 0 && py < image_size) {
                image.at<uchar>(py, px) = 255;
                // 점을 좀 더 두껍게 그리기
                cv::circle(image, cv::Point(px, py), 1, cv::Scalar(255), -1);
            }
        }
        
        // OpenCV HoughLines 적용
        std::vector<cv::Vec2f> cv_lines;
        cv::HoughLines(image, cv_lines, rho_resolution, theta_resolution, threshold);
        
        // OpenCV 결과를 HoughLine 구조체로 변환
        lines.clear();
        for (size_t i = 0; i < cv_lines.size() && i < 5; i++) {
            float img_rho = cv_lines[i][0];
            float img_theta = cv_lines[i][1];
            
            // 이미지 좌표계(u,v)에서 물리 좌표계(x,y)로 역변환
            // u = y, v = -x이므로 역변환은 x = -v, y = u
            // 따라서 theta와 rho도 이에 맞춰 변환
            double real_theta = img_theta; // 일단 그대로 써보고 결과 확인
            double real_rho = img_rho / scale;
            
            // 투표수는 추정값 (실제 OpenCV에서는 반환하지 않음)
            int estimated_votes = threshold + (int)(50 * (1.0 - i / std::max(1.0, (double)cv_lines.size())));
            
            lines.emplace_back(real_rho, real_theta, estimated_votes);
        }
        
    }
    
    // 3-2. 기존 커스텀 hough 변환 실시
    void performHoughTransform(const std::vector<ScanPoint>& map_data, std::vector<HoughLine>& lines)
    {
        if (map_data.empty()) return;
        
        // 범위 필터링 (로봇 중심 5x5미터)
        std::vector<ScanPoint> filtered_data = filterByRange(map_data);
        
        if (filtered_data.size() < 10) return;
        
        // Hough 변환 파라미터
        const int rho_resolution = 400;
        const int theta_resolution = 180;  // 0-180도
        const double max_rho = 10.0;
        
        // Accumulator 초기화
        std::vector<std::vector<int>> accumulator(rho_resolution, std::vector<int>(theta_resolution, 0));
        
        // Hough 변환 수행
        for (const auto& point : filtered_data)
        {
            for (int t = 0; t < theta_resolution; t++)
            {
                double theta = t * M_PI / 180.0;  // 0-180도
                double rho = point.x * cos(theta) + point.y * sin(theta);
                
                int rho_idx = (rho + max_rho) * rho_resolution / (2 * max_rho);
                
                if (rho_idx >= 0 && rho_idx < rho_resolution)
                {
                    accumulator[rho_idx][t]++;
                }
            }
        }
        
        // 피크 검출
        const int min_votes = std::max(10, (int)(filtered_data.size() * 0.1)); // 최소 10% 투표
        std::vector<std::tuple<int, int, int>> peaks; // (votes, rho_idx, theta_idx)
        
        for (int r = 2; r < rho_resolution - 2; r++)
        {
            for (int t = 2; t < theta_resolution - 2; t++)
            {
                if (accumulator[r][t] >= min_votes)
                {
                    // 로컬 최대값 확인
                    bool is_local_max = true;
                    for (int dr = -2; dr <= 2 && is_local_max; dr++)
                    {
                        for (int dt = -2; dt <= 2 && is_local_max; dt++)
                        {
                            if (accumulator[r + dr][t + dt] > accumulator[r][t])
                            {
                                is_local_max = false;
                            }
                        }
                    }
                    
                    if (is_local_max)
                    {
                        peaks.emplace_back(accumulator[r][t], r, t);
                    }
                }
            }
        }
        
        // 투표수 기준 정렬
        std::sort(peaks.begin(), peaks.end(), std::greater<>());
        
        // HoughLine으로 변환
        lines.clear();
        for (const auto& peak : peaks)
        {
            int votes = std::get<0>(peak);
            int rho_idx = std::get<1>(peak);
            int theta_idx = std::get<2>(peak);
            
            double rho = (rho_idx * 2 * max_rho / rho_resolution) - max_rho;
            double theta = theta_idx * M_PI / 180.0;
            
            lines.emplace_back(rho, theta, votes);
            
            if (lines.size() >= 5) break;
        }
        
    }
    
    // 범위 필터링 함수
    std::vector<ScanPoint> filterByRange(const std::vector<ScanPoint>& map_data)
    {
        std::vector<ScanPoint> filtered;
        
        double range_x = this->get_parameter("analysis_range_x").as_double();
        double range_y = this->get_parameter("analysis_range_y").as_double();
        
        try
        {
            // 로봇 위치 가져오기
            geometry_msgs::msg::TransformStamped robot_transform = tf_buffer_.lookupTransform(
                "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
            
            double robot_x = robot_transform.transform.translation.x;
            double robot_y = robot_transform.transform.translation.y;
            
            double half_x = range_x / 2.0;
            double half_y = range_y / 2.0;
            
            for (const auto& point : map_data)
            {
                double dx = point.x - robot_x;
                double dy = point.y - robot_y;
                
                if (std::abs(dx) <= half_x && std::abs(dy) <= half_y)
                {
                    filtered.push_back(point);
                }
            }
            
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Robot position failed, using all points: %s", ex.what());
            return map_data;
        }
        
        return filtered;
    }
    
    // 결과 출력 함수
    void displayResults(const std::vector<HoughLine>& lines, const std::string& source = "Scan")
    {
        bool use_opencv = this->get_parameter("use_opencv_hough").as_bool();
        std::string method = use_opencv ? "OpenCV Hough" : "Custom Hough";
        
        RCLCPP_INFO(this->get_logger(), "=== %s-based %s Transform 결과 ===", source.c_str(), method.c_str());
        
        for (size_t i = 0; i < lines.size() && i < 3; i++)
        {
            double angle_deg = lines[i].theta * 180.0 / M_PI;
            
            std::string line_type;
            if (angle_deg < 10 || angle_deg > 170) {
                line_type = " (수평선)";
            } else if (angle_deg > 80 && angle_deg < 100) {
                line_type = " (수직선)";
            } else {
                line_type = " (대각선)";
            }
            
            // 직선의 방정식 계산
            auto equation = calculateLineEquation(lines[i]);
            
            RCLCPP_INFO(this->get_logger(), "직선 %zu: 각도=%.1f°%s, 거리=%.2fm, 투표수=%d", 
                       i+1, angle_deg, line_type.c_str(), lines[i].rho, lines[i].votes);
            
            RCLCPP_INFO(this->get_logger(), "    방정식: %.3fx + %.3fy + %.3f = 0", 
                       equation.A, equation.B, equation.C);
            
            RCLCPP_INFO(this->get_logger(), "    기울기-절편 형태: %s", 
                       equation.slope_intercept.c_str());
        }
    }
    
    // 직선 방정식 구조체
    struct LineEquation {
        double A, B, C;  // Ax + By + C = 0
        std::string slope_intercept;  // y = mx + b 또는 x = c 형태
    };
    
    // 직선의 방정식 계산 함수
    LineEquation calculateLineEquation(const HoughLine& line)
    {
        LineEquation eq;
        
        // Hough 파라미터에서 일반형 방정식 계산
        // rho = x*cos(theta) + y*sin(theta)
        // 일반형: cos(theta)*x + sin(theta)*y - rho = 0
        
        eq.A = cos(line.theta);
        eq.B = sin(line.theta);
        eq.C = -line.rho;
        
        // 기울기-절편 형태 계산
        
        if (std::abs(eq.B) < 0.001) {
            // 수직선인 경우: x = c
            double x_intercept = -eq.C / eq.A;
            eq.slope_intercept = "x = " + std::to_string(x_intercept);
        } else {
            // 일반적인 경우: y = mx + b
            double slope = -eq.A / eq.B;  // 기울기 m = -A/B
            double y_intercept = -eq.C / eq.B;  // y절편 b = -C/B
            
            char buffer[100];
            if (std::abs(slope) < 0.001) {
                // 수평선인 경우
                snprintf(buffer, sizeof(buffer), "y = %.3f", y_intercept);
            } else {
                snprintf(buffer, sizeof(buffer), "y = %.3fx + %.3f", slope, y_intercept);
            }
            eq.slope_intercept = std::string(buffer);
        }
        
        return eq;
    }

    // 멤버 변수
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Map 데이터 및 결과 저장
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    std::vector<HoughLine> map_hough_lines_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapWallDetectorNode>());
    rclcpp::shutdown();
    return 0;
}