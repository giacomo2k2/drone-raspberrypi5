"""
Navigator node

    # Publish
    - local setpoints of the node (in ENU space) 


    # Subscribes to
    - local position of the drone (converts from GPS space to ENU space)


    # Actions
    - mission manager node calls an action on the navigator to perform a certain navigation mission
    - the navigator gets as an input from the mission manager the waypoints and holding time for each waypoints if present
    - the navigator gives a feedback of the drone following the path that is sending. (if something goes wrong the navigator stops the drone and then something is decided by the mission manager)
    - when the action is finished the navigator stops the drone and initiate the procedure for landing:
        current trajectory segmet
        progress along segment
        distance to next segment switch point
        deviation from trajectory

    # services
    - the navigator node at the beginning calls a service for generating the trajectory using the trajectory generator node. All the trajectory are given and saved in memory as segments with coefficients and times

    The navigator then internally handles feedback position to switch trajectory. It samples the trajectory and publish the setpoint in local space

"""

#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "drone_interfaces/srv/GenerateGlobalTrajectory.hpp"  
#include "drone_interfaces/action/FollowTrajectory.hpp" 
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

// ---------------- Trajectory segment ----------------
struct TrajectorySegment
{
    std::vector<double> coeffs_x;
    std::vector<double> coeffs_y;
    std::vector<double> coeffs_z;
    double start_time;
    double end_time;
};

// ---------------- Polynomial evaluation helper ----------------
double evalPoly(const std::vector<double> &coeffs, double t, int der_order=0)
{
    double val = 0.0;
    int n = coeffs.size();
    for (int i = der_order; i < n; ++i)
    {
        double term = coeffs[i];
        for (int k = 0; k < der_order; ++k)
            term *= (i - k);
        val += term * std::pow(t, i - der_order);
    }
    return val;
}

// ---------------- Navigator Node ----------------
class NavigatorNode : public rclcpp::Node
{
public:
    using FollowTrajectory = example_interfaces::action::FollowTrajectory;
    using GoalHandleFollowTrajectory = rclcpp_action::ServerGoalHandle<FollowTrajectory>;

    NavigatorNode() : Node("navigator_node")
    {
        // Publisher: MAVROS PositionTarget
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);

        // Subscriber: local drone position
        position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "local_position", 10,
            std::bind(&NavigatorNode::positionCallback, this, std::placeholders::_1)
        );

        // Action server: follow trajectory
        action_server_ = rclcpp_action::create_server<FollowTrajectory>(
            this,
            "follow_trajectory",
            std::bind(&NavigatorNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&NavigatorNode::handleCancel, this, std::placeholders::_1),
            std::bind(&NavigatorNode::handleAccepted, this, std::placeholders::_1)
        );

        // Service client: trajectory generation
        trajectory_client_ = this->create_client<example_interfaces::srv::Trigger>("generate_trajectory");

        RCLCPP_INFO(this->get_logger(), "Navigator node initialized");
    }

private:
    // ---------------- ROS interfaces ----------------
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp_action::Server<FollowTrajectory>::SharedPtr action_server_;
    rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr trajectory_client_;

    // ---------------- Trajectory storage ----------------
    std::vector<TrajectorySegment> trajectory_;
    size_t current_segment_ = 0;
    rclcpp::Time start_time_;

    // ---------------- Drone state ----------------
    geometry_msgs::msg::PoseStamped current_pose_;

    // ---------------- Drone position callback ----------------
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
    }

    // ---------------- Action callbacks ----------------
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FollowTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received new trajectory goal");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        RCLCPP_WARN(this->get_logger(), "Trajectory execution canceled!");
        stopDrone();
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        std::thread{std::bind(&NavigatorNode::executeTrajectory, this, goal_handle)}.detach();
    }

    // ---------------- Trajectory execution ----------------
    void executeTrajectory(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        // 1. Wait for trajectory service
        if (!trajectory_client_->wait_for_service(2s))
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory service unavailable");
            goal_handle->abort();
            return;
        }

        auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();
        auto result_future = trajectory_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) 
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory service call failed");
            goal_handle->abort();
            return;
        }

        // 2. Parse service response
        auto result = result_future.get();
        if (!result->success)
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory service reported failure");
            goal_handle->abort();
            return;
        }

        // TODO: Replace this with actual parsing from your custom service response
        // Example assumes you have arrays: result->coeffs_x, result->coeffs_y, result->coeffs_z, start_times, end_times
        trajectory_.clear();
        for (size_t i = 0; i < result->coeffs_x.size(); ++i)
        {
            TrajectorySegment seg;
            seg.coeffs_x = result->coeffs_x[i];
            seg.coeffs_y = result->coeffs_y[i];
            seg.coeffs_z = result->coeffs_z[i];
            seg.start_time = result->start_times[i];
            seg.end_time = result->end_times[i];
            trajectory_.push_back(seg);
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory received with %zu segments", trajectory_.size());

        start_time_ = this->now();
        current_segment_ = 0;

        auto feedback = std::make_shared<FollowTrajectory::Feedback>();
        auto action_result = std::make_shared<FollowTrajectory::Result>();
        rclcpp::Rate rate(50); // 50 Hz control loop

        while (rclcpp::ok() && current_segment_ < trajectory_.size())
        {
            double t = (this->now() - start_time_).seconds();

            TrajectorySegment &seg = trajectory_[current_segment_];
            double tau = (t - seg.start_time) / (seg.end_time - seg.start_time);
            tau = std::clamp(tau, 0.0, 1.0);

            mavros_msgs::msg::PositionTarget setpoint;
            setpoint.header.stamp = this->now();
            setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
            setpoint.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                                mavros_msgs::msg::PositionTarget::IGNORE_VY |
                                mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                                mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

            setpoint.position.x = evalPoly(seg.coeffs_x, tau);
            setpoint.position.y = evalPoly(seg.coeffs_y, tau);
            setpoint.position.z = evalPoly(seg.coeffs_z, tau);

            setpoint_pub_->publish(setpoint);

            // Publish feedback
            feedback->current_point.x = setpoint.position.x;
            feedback->current_point.y = setpoint.position.y;
            feedback->current_point.z = setpoint.position.z;
            goal_handle->publish_feedback(feedback);

            if (t >= seg.end_time)
                current_segment_++;

            rate.sleep();
        }

        stopDrone();
        goal_handle->succeed(action_result);
    }


    // ---------------- Stop drone safely ----------------
    void stopDrone()
    {
        RCLCPP_WARN(this->get_logger(), "Stopping drone!");
        mavros_msgs::msg::PositionTarget stop_point;
        stop_point.header.stamp = this->now();
        stop_point.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        stop_point.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
                               mavros_msgs::msg::PositionTarget::IGNORE_VY |
                               mavros_msgs::msg::PositionTarget::IGNORE_VZ |
                               mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                               mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                               mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                               mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                               mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        stop_point.position.x = current_pose_.pose.position.x;
        stop_point.position.y = current_pose_.pose.position.y;
        stop_point.position.z = current_pose_.pose.position.z;

        setpoint_pub_->publish(stop_point);
    }

};

// ---------------- Main ----------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

