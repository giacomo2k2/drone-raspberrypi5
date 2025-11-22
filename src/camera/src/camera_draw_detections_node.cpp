#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv2/opencv.hpp>
#include "image_transport/image_transport.hpp"
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>


#include <string>

using std::placeholders::_1;

class CameraDrawDetectionsNode : public rclcpp::Node
{
public:
    CameraDrawDetectionsNode() : Node("camera_draw_detections")
    {
        this->declare_parameter<std::string>("camera_name", "camera");
        camera_name_ = this->get_parameter("camera_name").as_string();

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();

        detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            camera_name_ + "/detections",
            qos,
            std::bind(&CameraDrawDetectionsNode::detectionsCallback, this, std::placeholders::_1)
        );

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_name_ + "/camera_info",
            qos,
            std::bind(&CameraDrawDetectionsNode::cameraInfoCallback, this, std::placeholders::_1)
        );

    }

    void init()
    {
    
        image_transport::ImageTransport it(shared_from_this());

        image_rect_sub_ = it.subscribe(
            camera_name_ + "/image_rect",
            1,  // queue size
            std::bind(&CameraDrawDetectionsNode::imageRectCallback, this, std::placeholders::_1)
        );

        image_detections_drawn_pub_ = it.advertise(
            camera_name_ + "/image_detections_drawn",
            1  // queue size
        );
    }


private:
    std::string camera_frame_id_;
    std::string camera_name_;

    image_transport::Subscriber image_rect_sub_;
    image_transport::Publisher image_detections_drawn_pub_;
    
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
    apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr latest_detections_;
    cv::Mat latest_frame_;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        latest_camera_info_ = msg;
    }

    void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        latest_detections_ = msg;
    }

    void imageRectCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        latest_frame_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        draw_detections();
    }

    void draw_detections() {
        if (latest_frame_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image frame available to draw detections.");
            return;
        }

        cv::Mat frame_with_detections = latest_frame_.clone();
        
        if (latest_camera_info_ && latest_detections_ && !latest_detections_->detections.empty()) {
            
    
            for (const auto& detection : latest_detections_->detections) {
                // Draw bounding box
                std::vector<cv::Point> corners;
                for (const auto& corner : detection.corners) {
                    corners.emplace_back(corner.x, corner.y);
                }
                for (size_t i = 0; i < corners.size(); ++i) {
                    cv::line(frame_with_detections, corners[i], corners[(i + 1) % corners.size()], cv::Scalar(0, 255, 0), 2);
                }    
                
            }
        }

        // Publish the image with detections drawn
        sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
            latest_camera_info_->header,
            "bgr8",
            frame_with_detections
        ).toImageMsg();

        image_detections_drawn_pub_.publish(output_msg);

    }

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraDrawDetectionsNode>();
    node->init();  
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}