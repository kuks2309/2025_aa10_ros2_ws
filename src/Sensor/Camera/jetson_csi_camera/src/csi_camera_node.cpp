#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

class CSICameraNode : public rclcpp::Node
{
public:
    CSICameraNode() : Node("csi_camera_node")
    {
        // Declare parameters
        this->declare_parameter<int>("camera_id", 0);
        this->declare_parameter<int>("image_width", 1280);
        this->declare_parameter<int>("image_height", 720);
        this->declare_parameter<int>("framerate", 30);
        this->declare_parameter<int>("flip_method", 0);
        this->declare_parameter<std::string>("camera_frame_id", "camera_link");
        this->declare_parameter<double>("publish_rate", 30.0);
        this->declare_parameter<double>("image_scale", 1.0);  // Image scaling factor (0.5 = 50%, 1.0 = 100%)

        // Get parameters
        camera_id_ = this->get_parameter("camera_id").as_int();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();
        framerate_ = this->get_parameter("framerate").as_int();
        flip_method_ = this->get_parameter("flip_method").as_int();
        camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        image_scale_ = this->get_parameter("image_scale").as_double();

        // Create publisher
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

        // Build GStreamer pipeline for CSI camera
        std::string gst_pipeline = gstreamer_pipeline(
            camera_id_,
            image_width_,
            image_height_,
            framerate_,
            flip_method_
        );

        RCLCPP_INFO(this->get_logger(), "Opening CSI camera with pipeline:");
        RCLCPP_INFO(this->get_logger(), "%s", gst_pipeline.c_str());

        // Open camera
        cap_.open(gst_pipeline, cv::CAP_GSTREAMER);

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "====================================");
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSI camera!");
            RCLCPP_ERROR(this->get_logger(), "Camera ID: %d", camera_id_);
            RCLCPP_ERROR(this->get_logger(), "Please check:");
            RCLCPP_ERROR(this->get_logger(), "1. CSI camera is properly connected");
            RCLCPP_ERROR(this->get_logger(), "2. GStreamer is installed");
            RCLCPP_ERROR(this->get_logger(), "3. Camera permissions are set");
            RCLCPP_ERROR(this->get_logger(), "====================================");
            throw std::runtime_error("Failed to open CSI camera");
        }

        RCLCPP_INFO(this->get_logger(), "====================================");
        RCLCPP_INFO(this->get_logger(), "CSI Camera opened successfully!");
        RCLCPP_INFO(this->get_logger(), "Camera ID: %d", camera_id_);
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", image_width_, image_height_);
        RCLCPP_INFO(this->get_logger(), "Framerate: %d fps", framerate_);
        RCLCPP_INFO(this->get_logger(), "Image Scale: %.2fx (Output: %dx%d)",
                    image_scale_,
                    static_cast<int>(image_width_ * image_scale_),
                    static_cast<int>(image_height_ * image_scale_));
        RCLCPP_INFO(this->get_logger(), "====================================");

        // Create timer for publishing
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&CSICameraNode::timer_callback, this));
    }

    ~CSICameraNode()
    {
        if (cap_.isOpened())
        {
            cap_.release();
            RCLCPP_INFO(this->get_logger(), "CSI camera released");
        }
    }

private:
    std::string gstreamer_pipeline(int camera_id, int capture_width, int capture_height,
                                   int framerate, int flip_method)
    {
        return "nvarguscamerasrc sensor-id=" + std::to_string(camera_id) +
               " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) +
               ", height=(int)" + std::to_string(capture_height) +
               ", framerate=(fraction)" + std::to_string(framerate) + "/1" +
               " ! nvvidconv flip-method=" + std::to_string(flip_method) +
               " ! video/x-raw, width=(int)" + std::to_string(capture_width) +
               ", height=(int)" + std::to_string(capture_height) +
               ", format=(string)BGRx" +
               " ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    }

    void timer_callback()
    {
        cv::Mat frame;

        if (!cap_.read(frame))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame from CSI camera");
            return;
        }

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }

        // Apply image scaling if scale factor is not 1.0
        cv::Mat output_frame = frame;
        if (image_scale_ != 1.0 && image_scale_ > 0.0)
        {
            int scaled_width = static_cast<int>(frame.cols * image_scale_);
            int scaled_height = static_cast<int>(frame.rows * image_scale_);

            cv::resize(frame, output_frame, cv::Size(scaled_width, scaled_height),
                      0, 0, cv::INTER_LINEAR);

            if (frame_count_ == 0)
            {
                RCLCPP_INFO(this->get_logger(),
                           "Scaling image from %dx%d to %dx%d (scale: %.2f)",
                           frame.cols, frame.rows,
                           scaled_width, scaled_height,
                           image_scale_);
            }
        }

        // Convert OpenCV image to ROS message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output_frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = camera_frame_id_;

        // Publish image
        image_pub_->publish(*msg);

        // Publish camera info
        auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        camera_info_msg->header = msg->header;
        camera_info_msg->width = output_frame.cols;
        camera_info_msg->height = output_frame.rows;
        camera_info_msg->distortion_model = "plumb_bob";

        camera_info_pub_->publish(*camera_info_msg);

        frame_count_++;

        if (frame_count_ % 30 == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Published %ld frames", frame_count_);
        }
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int camera_id_;
    int image_width_;
    int image_height_;
    int framerate_;
    int flip_method_;
    std::string camera_frame_id_;
    double image_scale_;
    size_t frame_count_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<CSICameraNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("csi_camera"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
