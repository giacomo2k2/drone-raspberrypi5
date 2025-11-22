#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include "drone_interfaces/srv/stop_recording.hpp"
#include <image_transport/image_transport.hpp>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;
using std::placeholders::_1;
using std::placeholders::_2;

class LoggerNode : public rclcpp::Node
{
public:
    LoggerNode() : Node("logger_node")
    {
        // Parameters
        this->declare_parameter<std::string>("camera_name", "camera");
        this->get_parameter("camera_name", camera_name_);

        // Create video folder
        fs::path video_folder("data/video");
        fs::create_directories(video_folder);

        // Timestamped filename
        auto t = std::chrono::system_clock::now();
        auto t_c = std::chrono::system_clock::to_time_t(t);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&t_c), "%Y_%m_%d_%H_%M_%S");
        video_path_ = video_folder / (ss.str() + ".mp4");

        // Open VideoWriter
        int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1'); // H264
        fps_ = 30.0;
        frame_size_ = cv::Size(640, 480);
        video_writer_.open(video_path_.string(), fourcc, fps_, frame_size_);

        if (!video_writer_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Recording video to: %s", video_path_.string().c_str());
        }

    
        // Service
        stop_service_ = this->create_service<drone_interfaces::srv::StopRecording>(
            "drone/video_logger/stop_recording",
            std::bind(&LoggerNode::stop_callback, this, _1, _2)
        );
    }

    void init()
    {
    
        image_transport::ImageTransport it(shared_from_this());

        image_detections_drawn_sub_ = it.subscribe(
            camera_name_ + "/image_detections_drawn",
            1,  // queue size
            std::bind(&LoggerNode::image_callback, this, std::placeholders::_1)
        );
    

    }

    ~LoggerNode()
    {
        if (video_writer_.isOpened())
        {
            video_writer_.release();
        }
    }

private:

    std::string camera_name_;
    fs::path video_path_;
    cv::VideoWriter video_writer_;
    cv::Size frame_size_;
    double fps_;

    image_transport::Subscriber image_detections_drawn_sub_;
    rclcpp::Service<drone_interfaces::srv::StopRecording>::SharedPtr stop_service_;

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Optional resize
            // cv::resize(frame, frame, frame_size_);

            video_writer_.write(frame);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CvBridge exception: %s", e.what());
        }
    }

    void stop_callback(
        const std::shared_ptr<drone_interfaces::srv::StopRecording::Request> request,
        std::shared_ptr<drone_interfaces::srv::StopRecording::Response> response)
    {
        (void)request; // unused

        if (video_writer_.isOpened())
        {
            video_writer_.release();
            RCLCPP_INFO(this->get_logger(), "Video recording stopped safely.");
            response->success = true;
            response->message = "Video recording stopped.";
        }
        else
        {
            response->success = false;
            response->message = "Video was not recording.";
        }
    }

   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LoggerNode>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
