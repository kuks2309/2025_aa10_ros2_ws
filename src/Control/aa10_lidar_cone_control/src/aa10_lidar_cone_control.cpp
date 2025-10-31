#define DEBUG 1
#define DEBUG_ROS_INFO 1

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>

#include <string.h>
#include <iostream>
#include <sstream>
#include <cmath>

#include <opencv2/opencv.hpp>

// unit : m

#define OFF 0
#define ON  1

#define IMG_HEIGHT_LIDAR  500
#define IMG_WIDTH_LIDAR   400
#define MAX_CON_NO        200
#define IMAGE_SCALE       400


using namespace std;
using namespace cv;

typedef struct
{
  float x;  // x
  float y;  // y
  float z;  // z
  float i;  // intensity
  uint16_t r;  // ring
} Point_3D_lidar;

class LidarConeControl : public rclcpp::Node
{
public:
    LidarConeControl() : Node("lidar_cone_control_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("lidar2d_cloud_topic", "/pointcloud2d");
        this->declare_parameter<std::string>("imu_heading_angle_topic", "handsfree/imu/yaw_degree");
        this->declare_parameter<int>("min_blob_area", 250);
        this->declare_parameter<int>("max_blob_area", 600);
        this->declare_parameter<double>("lidar_offset_y", 0.0);
        this->declare_parameter<int>("image_scale", 400);
        this->declare_parameter<double>("cone_spacing_min", 0.15);
        this->declare_parameter<double>("cone_spacing_max", 0.35);

        // Get parameters
        std::string lidar2d_cloud_topic = this->get_parameter("lidar2d_cloud_topic").as_string();
        std::string imu_heading_angle_topic = this->get_parameter("imu_heading_angle_topic").as_string();
        min_blob_area_ = this->get_parameter("min_blob_area").as_int();
        max_blob_area_ = this->get_parameter("max_blob_area").as_int();
        lidar_offset_y_ = this->get_parameter("lidar_offset_y").as_double();
        image_scale_ = this->get_parameter("image_scale").as_int();
        cone_spacing_min_ = this->get_parameter("cone_spacing_min").as_double();
        cone_spacing_max_ = this->get_parameter("cone_spacing_max").as_double();

        // Initialize subscribers
        sub_heading_angle_ = this->create_subscription<std_msgs::msg::Float32>(
            imu_heading_angle_topic, 10,
            std::bind(&LidarConeControl::heading_angle_callback, this, std::placeholders::_1));

        sub_2d_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar2d_cloud_topic, 10,
            std::bind(&LidarConeControl::lidar2d_pointcloud2_callback, this, std::placeholders::_1));

        // Initialize publishers
        lidar_xte_pub_ = this->create_publisher<std_msgs::msg::Float32>("/xte/lidar_cone", 1);
        cone_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_cone/markers", 1);

        // Initialize variables
        yaw_d_ = 0.0;
        cone_left_cnt_ = 0;
        cone_right_cnt_ = 0;

        // Create OpenCV windows
        cv::namedWindow("Lidar window", cv::WINDOW_NORMAL);
        cv::resizeWindow("Lidar window", IMG_WIDTH_LIDAR/2, IMG_HEIGHT_LIDAR/2);
        cv::moveWindow("Lidar window", 100, 500);

        cv::namedWindow("result", cv::WINDOW_NORMAL);
        cv::resizeWindow("result", IMG_WIDTH_LIDAR/2, IMG_HEIGHT_LIDAR/2);

        RCLCPP_INFO(this->get_logger(), "LiDAR Cone Control Node Started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", lidar2d_cloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Blob area range: [%d, %d]", min_blob_area_, max_blob_area_);
    }

    ~LidarConeControl()
    {
        cv::destroyAllWindows();
    }

private:
    // ROS2 subscribers and publishers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_heading_angle_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_2d_lidar_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lidar_xte_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cone_markers_pub_;

    // Parameters
    int min_blob_area_;
    int max_blob_area_;
    double lidar_offset_y_;
    int image_scale_;
    double cone_spacing_min_;
    double cone_spacing_max_;

    // State variables
    double yaw_d_;
    geometry_msgs::msg::Pose2D cone_left_pose2d_[30];
    geometry_msgs::msg::Pose2D cone_right_pose2d_[30];
    int cone_left_cnt_;
    int cone_right_cnt_;

    // OpenCV images
    cv::Mat image_obstacle_;
    cv::Mat image_obstacle_color_;
    cv::Mat image_dilate_;

    void publish_cone_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // Add left cone markers (yellow circles)
        for (int i = 0; i <= cone_left_cnt_; i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "left_cones";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Convert image coordinates to world coordinates
            float world_y = (cone_left_pose2d_[i].x - IMG_WIDTH_LIDAR/2) / (float)image_scale_;
            float world_x = (IMG_HEIGHT_LIDAR - cone_left_pose2d_[i].y) / (float)image_scale_;

            marker.pose.position.x = world_x;
            marker.pose.position.y = world_y;
            marker.pose.position.z = 0.0;  // Height 0
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.1;  // Diameter
            marker.scale.y = 0.1;  // Diameter
            marker.scale.z = 0.01; // Very thin cylinder (circle appearance)

            // Yellow color for left cones
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);

            marker_array.markers.push_back(marker);
        }

        // Add left cone connection lines
        for (int i = 0; i < cone_left_cnt_; i++)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "base_link";
            line.header.stamp = this->now();
            line.ns = "left_cone_lines";
            line.id = i;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p1, p2;
            p1.y = (cone_left_pose2d_[i].x - IMG_WIDTH_LIDAR/2) / (float)image_scale_;
            p1.x = (IMG_HEIGHT_LIDAR - cone_left_pose2d_[i].y) / (float)image_scale_;
            p1.z = 0.0;

            p2.y = (cone_left_pose2d_[i+1].x - IMG_WIDTH_LIDAR/2) / (float)image_scale_;
            p2.x = (IMG_HEIGHT_LIDAR - cone_left_pose2d_[i+1].y) / (float)image_scale_;
            p2.z = 0.0;

            line.points.push_back(p1);
            line.points.push_back(p2);

            line.scale.x = 0.02;  // Line width

            line.color.r = 1.0;
            line.color.g = 1.0;
            line.color.b = 0.0;
            line.color.a = 0.8;

            line.lifetime = rclcpp::Duration::from_seconds(0.5);

            marker_array.markers.push_back(line);
        }

        // Add right cone markers (blue circles)
        for (int i = 0; i <= cone_right_cnt_; i++)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link";
            marker.header.stamp = this->now();
            marker.ns = "right_cones";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Convert image coordinates to world coordinates
            float world_y = (cone_right_pose2d_[i].x - IMG_WIDTH_LIDAR/2) / (float)image_scale_;
            float world_x = (IMG_HEIGHT_LIDAR - cone_right_pose2d_[i].y) / (float)image_scale_;

            marker.pose.position.x = world_x;
            marker.pose.position.y = world_y;
            marker.pose.position.z = 0.0;  // Height 0
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.1;  // Diameter
            marker.scale.y = 0.1;  // Diameter
            marker.scale.z = 0.01; // Very thin cylinder (circle appearance)

            // Blue color for right cones
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);

            marker_array.markers.push_back(marker);
        }

        // Add right cone connection lines
        for (int i = 0; i < cone_right_cnt_; i++)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "base_link";
            line.header.stamp = this->now();
            line.ns = "right_cone_lines";
            line.id = i;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p1, p2;
            p1.y = (cone_right_pose2d_[i].x - IMG_WIDTH_LIDAR/2) / (float)image_scale_;
            p1.x = (IMG_HEIGHT_LIDAR - cone_right_pose2d_[i].y) / (float)image_scale_;
            p1.z = 0.0;

            p2.y = (cone_right_pose2d_[i+1].x - IMG_WIDTH_LIDAR/2) / (float)image_scale_;
            p2.x = (IMG_HEIGHT_LIDAR - cone_right_pose2d_[i+1].y) / (float)image_scale_;
            p2.z = 0.0;

            line.points.push_back(p1);
            line.points.push_back(p2);

            line.scale.x = 0.02;  // Line width

            line.color.r = 0.0;
            line.color.g = 0.0;
            line.color.b = 1.0;
            line.color.a = 0.8;

            line.lifetime = rclcpp::Duration::from_seconds(0.5);

            marker_array.markers.push_back(line);
        }

        // Only publish if we have markers
        if (!marker_array.markers.empty())
        {
            cone_markers_pub_->publish(marker_array);
            RCLCPP_DEBUG(this->get_logger(), "Published %zu markers", marker_array.markers.size());
        }
    }

    void bubble_sort(geometry_msgs::msg::Pose2D cone_pos2d_array[], int n)
    {
        geometry_msgs::msg::Pose2D cone_pos2d_temp;
        for (int i = 0; i < n - 1; i++)
        {
            for (int j = 0; j < n - i - 1; j++)
            {
                // Sort by y coordinate (distance from robot)
                if (cone_pos2d_array[j].y < cone_pos2d_array[j + 1].y) {
                    // Swap elements
                    cone_pos2d_temp = cone_pos2d_array[j];
                    cone_pos2d_array[j] = cone_pos2d_array[j + 1];
                    cone_pos2d_array[j + 1] = cone_pos2d_temp;
                }
            }
        }
    }

    void find_cone_left_pos(geometry_msgs::msg::Pose2D cone_pos[], int cnt)
    {
        if(cnt <= 2) return;

        RCLCPP_INFO(this->get_logger(), "find_cone_left: cnt=%d, cone_left_cnt=%d, ref_pos=(%.1f, %.1f)",
                    cnt, cone_left_cnt_, cone_left_pose2d_[cone_left_cnt_].x, cone_left_pose2d_[cone_left_cnt_].y);

        for(int i = 2;  i < cnt ; i++)
        {
            int best_j = -1;
            double min_angle_diff = 180.0;  // Start with max angle

            for(int j = i; j < cnt ; j++)
            {
                double dis_x    = (cone_pos[j].x - cone_left_pose2d_[cone_left_cnt_].x);
                double dis_y    = (cone_pos[j].y - cone_left_pose2d_[cone_left_cnt_].y);
                double distance = sqrt(dis_x*dis_x + dis_y*dis_y) / image_scale_;  // Convert to meters

                RCLCPP_INFO(this->get_logger(), "  candidate[%d]: pos=(%.1f,%.1f) dist=%.3fm",
                            j, cone_pos[j].x, cone_pos[j].y, distance);

                // Check distance range first
                if((distance >= cone_spacing_min_) && (distance <= cone_spacing_max_))
                {
                    // Calculate angle from forward direction (Y-axis, upward in image)
                    // Forward direction is -Y (toward top of image, away from robot)
                    double angle_from_forward = atan2(fabs(dis_x), fabs(dis_y)) * 180.0 / M_PI;

                    RCLCPP_INFO(this->get_logger(), "    -> PASS dist check! angle=%.1f°", angle_from_forward);

                    // Select cone with smallest angle deviation (most aligned with forward direction)
                    if(angle_from_forward < min_angle_diff)
                    {
                        min_angle_diff = angle_from_forward;
                        best_j = j;
                    }
                }
            }

            if(best_j != -1)
            {
                cone_left_cnt_++;
                cone_left_pose2d_[cone_left_cnt_] = cone_pos[best_j];
                RCLCPP_INFO(this->get_logger(), "  -> Selected cone[%d] at (%.1f,%.1f), new cnt=%d",
                            best_j, cone_pos[best_j].x, cone_pos[best_j].y, cone_left_cnt_);
            }
        }

        // Draw left cones
        for(int i = 0 ; i <= cone_left_cnt_; i++)
        {
            circle(image_obstacle_color_, cv::Point(cone_left_pose2d_[i].x, cone_left_pose2d_[i].y), 10, cv::Scalar(0, 255, 255), 2, 8, 0);
        }

        // Draw lines between consecutive left cones
        for(int i = 0 ; i <= cone_left_cnt_-1; i++)
        {
            line(image_obstacle_color_, cv::Point(cone_left_pose2d_[i].x, cone_left_pose2d_[i].y),
                 cv::Point(cone_left_pose2d_[i+1].x, cone_left_pose2d_[i+1].y), cv::Scalar(0, 255, 255), 2, 8, 0);
        }
    }

    void find_cone_right_pos(geometry_msgs::msg::Pose2D cone_pos[], int cnt)
    {
        if(cnt <= 2) return;

        for(int i = 2;  i < cnt ; i++)
        {
            int best_j = -1;
            double min_angle_diff = 180.0;  // Start with max angle

            for(int j = i; j < cnt ; j++)
            {
                double dis_x    = (cone_pos[j].x - cone_right_pose2d_[cone_right_cnt_].x);
                double dis_y    = (cone_pos[j].y - cone_right_pose2d_[cone_right_cnt_].y);
                double distance = sqrt(dis_x*dis_x + dis_y*dis_y) / image_scale_;  // Convert to meters

                // Check distance range first
                if((distance >= cone_spacing_min_) && (distance <= cone_spacing_max_))
                {
                    // Calculate angle from forward direction (Y-axis, upward in image)
                    // Forward direction is -Y (toward top of image, away from robot)
                    double angle_from_forward = atan2(fabs(dis_x), fabs(dis_y)) * 180.0 / M_PI;

                    // Select cone with smallest angle deviation (most aligned with forward direction)
                    if(angle_from_forward < min_angle_diff)
                    {
                        min_angle_diff = angle_from_forward;
                        best_j = j;
                    }
                }
            }

            if(best_j != -1)
            {
                cone_right_cnt_++;
                cone_right_pose2d_[cone_right_cnt_] = cone_pos[best_j];
            }
        }

        // Draw right cones
        for(int i = 0 ; i <= cone_right_cnt_; i++)
        {
            circle(image_obstacle_color_, cv::Point(cone_right_pose2d_[i].x, cone_right_pose2d_[i].y), 10, cv::Scalar(255, 0, 0), 2, 8, 0);
        }

        // Draw lines between consecutive right cones
        for(int i = 0 ; i <= cone_right_cnt_-1; i++)
        {
            line(image_obstacle_color_, cv::Point(cone_right_pose2d_[i].x, cone_right_pose2d_[i].y),
                 cv::Point(cone_right_pose2d_[i+1].x, cone_right_pose2d_[i+1].y), cv::Scalar(255, 0, 0), 2, 8, 0);
        }
    }

    void init_cone_blob_pos(geometry_msgs::msg::Pose2D cone_pos2d_array[], int cnt)
    {
        double lidar_xte = 0;

        // Draw center line
        line(image_obstacle_color_, cv::Point(IMG_WIDTH_LIDAR/2, 0), cv::Point(IMG_WIDTH_LIDAR/2, IMG_HEIGHT_LIDAR),
             cv::Scalar(147, 20, 255), 3, 8, 0);

        if((cnt >= 2) && (cone_pos2d_array[0].y >= IMG_HEIGHT_LIDAR * 0.7))
        {
            // Identify first cone (left or right)
            if(cone_pos2d_array[0].x < IMG_WIDTH_LIDAR/2)
            {
                cone_left_pose2d_[0] = cone_pos2d_array[0];
                cone_left_cnt_++;
                circle(image_obstacle_color_, cv::Point(cone_left_pose2d_[0].x, cone_left_pose2d_[0].y), 10, cv::Scalar(0, 255, 255), 2, 8, 0);
            }
            else
            {
                cone_right_pose2d_[0] = cone_pos2d_array[0];
                cone_right_cnt_++;
                circle(image_obstacle_color_, cv::Point(cone_right_pose2d_[0].x, cone_right_pose2d_[0].y), 10, cv::Scalar(255, 0, 0), 2, 8, 0);
            }

            // Identify second cone (left or right)
            if(cone_pos2d_array[1].x > IMG_WIDTH_LIDAR/2)
            {
                cone_right_pose2d_[0] = cone_pos2d_array[1];
                cone_right_cnt_++;
                circle(image_obstacle_color_, cv::Point(cone_right_pose2d_[0].x, cone_right_pose2d_[0].y), 10, cv::Scalar(255, 0, 0), 2, 8, 0);
            }
            else
            {
                cone_left_pose2d_[0] = cone_pos2d_array[1];
                cone_left_cnt_++;
                circle(image_obstacle_color_, cv::Point(cone_left_pose2d_[0].x, cone_left_pose2d_[0].y), 10, cv::Scalar(0, 255, 255), 2, 8, 0);
            }

            // Check left-right distance constraint (50-70cm)
            if(cone_left_cnt_ > 0 && cone_right_cnt_ > 0)
            {
                double dx = cone_right_pose2d_[0].x - cone_left_pose2d_[0].x;
                double dy = cone_right_pose2d_[0].y - cone_left_pose2d_[0].y;
                double lateral_distance = sqrt(dx*dx + dy*dy) / image_scale_;

                RCLCPP_INFO(this->get_logger(), "Initial cone pair lateral distance: %.3fm", lateral_distance);

                if(lateral_distance >= 0.50 && lateral_distance <= 0.70)
                {
                    // Draw line between first left and right cones
                    line(image_obstacle_color_, cv::Point(cone_left_pose2d_[0].x, cone_left_pose2d_[0].y),
                         cv::Point(cone_right_pose2d_[0].x, cone_right_pose2d_[0].y), cv::Scalar(0, 255, 0), 3, 8, 0);

                    // Calculate cross-track error (using first cone pair)
                    lidar_xte = IMG_WIDTH_LIDAR/2 - (cone_left_pose2d_[0].x + cone_right_pose2d_[0].x)/2;

                    std_msgs::msg::Float32 xte_lidar_msg;
                    xte_lidar_msg.data = lidar_xte;
                    lidar_xte_pub_->publish(xte_lidar_msg);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Initial cone pair rejected: lateral distance %.3fm not in range [0.50, 0.70]", lateral_distance);
                    cone_left_cnt_ = 0;
                    cone_right_cnt_ = 0;
                }
            }
        }
        cone_left_cnt_ = 0;
        cone_right_cnt_ = 0;
    }

    int lidar_cone_blob_detection(cv::Mat mat_input)
    {
        int result = 1;
        cv::Mat img_labels, stats, centroids;
        geometry_msgs::msg::Pose2D cone_pose2d[50];

        int numOfLabels = cv::connectedComponentsWithStats(mat_input, img_labels, stats, centroids, 8, CV_32S);

        int cnt = 0;
        for (int i = 1; i < numOfLabels; i++)
        {
            int area  = stats.at<int>(i, cv::CC_STAT_AREA);
            int* p    = stats.ptr<int>(i);

            if((area >= min_blob_area_) && (area <= max_blob_area_))
            {
                cone_pose2d[cnt].x = p[0] + p[2]/2;
                cone_pose2d[cnt].y = p[1] + p[3]/2;
                cnt++;
            }
        }

        if(cnt >= 2)
        {
            bubble_sort(cone_pose2d, cnt);
        }

        init_cone_blob_pos(cone_pose2d, cnt);
        find_cone_left_pos(cone_pose2d, cnt);
        find_cone_right_pos(cone_pose2d, cnt);

        // Publish cone markers to RViz
        publish_cone_markers();

        RCLCPP_INFO(this->get_logger(), "cone cnt[%2d %2d]", cone_left_cnt_, cone_right_cnt_);
        return result;
    }

    void heading_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        yaw_d_ = msg->data;
    }

    void lidar2d_pointcloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr scan)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*scan, *cloud);

        pcl::PCLPointCloud2 cloud_p;
        pcl::toPCLPointCloud2(*cloud, cloud_p);

        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_p, output);
        output.header.frame_id = "lidar_scan";

        pcl::PointCloud<pcl::PointXYZ> _2d_cloud;
        pcl::fromROSMsg(output, _2d_cloud);

        image_obstacle_       = cv::Mat::zeros(IMG_HEIGHT_LIDAR, IMG_WIDTH_LIDAR, CV_8UC1);
        image_obstacle_color_ = cv::Mat::zeros(IMG_HEIGHT_LIDAR, IMG_WIDTH_LIDAR, CV_8UC3);

        for(unsigned int j = 0; j < _2d_cloud.points.size(); j++)
        {
            int img_x, img_y;

            // FIXED BUG: Changed _2d_cloud[j].y to _2d_cloud.points[j].y
            if((_2d_cloud.points[j].x != 0.0) && (_2d_cloud.points[j].y != 0.0))
            {
                img_x = IMG_WIDTH_LIDAR/2 + (int)(_2d_cloud.points[j].y * image_scale_);
                img_y = IMG_HEIGHT_LIDAR - (int)(_2d_cloud.points[j].x * image_scale_);

                if(((img_x >= 0) && (img_x < IMG_WIDTH_LIDAR)) && ((img_y >= 0) && (img_y < IMG_HEIGHT_LIDAR)))
                {
                    image_obstacle_.at<uchar>(img_y, img_x) = 255;
                }
            }
        }

        // Apply dilation
        dilate(image_obstacle_, image_dilate_, cv::Mat::ones(cv::Size(7, 7), CV_8UC1), cv::Point(-1, -1), 2);

        // Detect cones
        lidar_cone_blob_detection(image_dilate_);

        cv::imshow("Lidar window", image_dilate_);
        cv::imshow("result", image_obstacle_color_);
        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarConeControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
