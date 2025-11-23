#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <drone_interfaces/msg/apriltag_position.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <Eigen/Dense>
#include <chrono>

using namespace std::chrono_literals;

class ApriltagGuidanceNode : public rclcpp::Node
{
public:
    ApriltagGuidanceNode() : Node("apriltag_guidance_node")
    {
        // Parameters
        this->declare_parameter<std::string>("camera_frame", "camera");
        camera_frame_ = this->get_parameter("camera_frame").as_string();

        this->declare_parameter<std::string>("camera_name", "camera");
        camera_name_ = this->get_parameter("camera_name").as_string();

        this->declare_parameter<std::string>("platform", "CC");
        platform_ = this->get_parameter("platform").as_string();

        z_rot_ = M_PI;  // both cases in Python code are the same

        // TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        apriltag_distance_pub_ = this->create_publisher<drone_interfaces::msg::ApriltagPosition>(
            "/drone/apriltag_guidance/apriltag_distance", 10);

        // Subscribers
        detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/" + camera_name_ + "/detections", rclcpp::QoS(5).best_effort(),
            std::bind(&ApriltagGuidanceNode::tags_callback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/" + camera_name_ + "/camera_info", rclcpp::QoS(5).best_effort(),
            std::bind(&ApriltagGuidanceNode::info_callback, this, std::placeholders::_1));

        heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/fc_interface/heading", 10,
            std::bind(&ApriltagGuidanceNode::heading_callback, this, std::placeholders::_1));

        // Timer
        timer_ = this->create_wall_timer(10ms, std::bind(&ApriltagGuidanceNode::main_loop, this));

        start_time_ = this->now();
    }

private:
    // Parameters
    std::string camera_frame_;
    std::string camera_name_;
    std::string platform_;
    double z_rot_;

    // Subscribers & Publishers
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
    rclcpp::Publisher<drone_interfaces::msg::ApriltagPosition>::SharedPtr apriltag_distance_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // State
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    std::vector<apriltag_msgs::msg::AprilTagDetection> detections_;
    double heading_;
    Eigen::Vector3d center_cam_;
    bool center_cam_valid_ = false;
    rclcpp::Time start_time_;
    double timeout_ = 1.0;

    void main_loop()
    {
        if (!center_cam_valid_)
            return;

        auto msg = drone_interfaces::msg::ApriltagPosition();
        msg.header.stamp = this->now();
        msg.center.x = center_cam_(0);
        msg.center.y = center_cam_(1);
        msg.center.z = center_cam_(2);
        msg.timeout_expired = (this->now() - start_time_).seconds() > timeout_;

        apriltag_distance_pub_->publish(msg);
    }

    void heading_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        heading_ = msg->data;
    }

    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        camera_info_ = msg;
    }

    void tags_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        detections_ = msg->detections;
        if (!camera_info_ || detections_.empty())
            return;

        start_time_ = this->now();

        for (auto &det : detections_)
        {
            geometry_msgs::msg::TransformStamped t;
            try
            {
                t = tf_buffer_->lookupTransform(
                    camera_frame_, "tag36h11:" + std::to_string(det.id),
                    tf2::TimePointZero, tf2::durationFromSec(0.5));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed for tag %d: %s", det.id, ex.what());
                continue;
            }

            Eigen::Quaterniond q(t.transform.rotation.w, t.transform.rotation.x,
                                 t.transform.rotation.y, t.transform.rotation.z);
            Eigen::Matrix3d R_mat = q.toRotationMatrix();
            Eigen::Vector3d t_vec(t.transform.translation.x,
                                  t.transform.translation.y,
                                  t.transform.translation.z);

            Eigen::AngleAxisd R_z_rot(z_rot_, Eigen::Vector3d::UnitZ());
            R_mat = R_z_rot.toRotationMatrix() * R_mat;
            t_vec = R_z_rot.toRotationMatrix() * t_vec;

            if (!std::isnan(heading_))
            {
                double heading_rad = heading_ * M_PI / 180.0;
                Eigen::AngleAxisd R_heading(heading_rad, Eigen::Vector3d::UnitZ());
                R_mat = R_heading.toRotationMatrix() * R_mat;
                t_vec = R_heading.toRotationMatrix() * t_vec;
            }

            Eigen::Vector3d center_3d(0, 0, 0);
            center_cam_ = R_mat * center_3d + t_vec;
            center_cam_valid_ = true;
        }
    }

    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApriltagGuidanceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
