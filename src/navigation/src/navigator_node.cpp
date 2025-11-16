#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <thread>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/position_target.hpp"

#include "drone_interfaces/srv/generate_global_trajectory.hpp"
#include "drone_interfaces/srv/takeoff.hpp"
#include "drone_interfaces/srv/land.hpp"

#include "drone_interfaces/action/follow_trajectory.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

#include "drone_interfaces/msg/apriltag_position.hpp"

#include "drone_interfaces/srv/activate_detector.hpp"

#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;


enum class SegmentType 
{
    Takeoff,
    Fly,
    Land, 
    Hold, 
};

inline std::string toString(SegmentType type) {
    switch (type) {
        case SegmentType::Takeoff: return "Takeoff";
        case SegmentType::Fly:     return "Fly";
        case SegmentType::Land:    return "Land";
        case SegmentType::Hold:    return "Hold";
        default:                   return "Unknown";
    }
}

// ---------------- Trajectory segment ----------------
struct TrajectorySegment
{
    SegmentType type;

    // Takeoff
    double takeoff_altitude = 0.0;

    // Fly
    std::vector<double> coeffs_x;
    std::vector<double> coeffs_y;
    std::vector<double> coeffs_z;
    uint32_t num_segments = 0;
    uint32_t num_coeffs_segment = 0;
    std::vector<double> periods; 

    // Hold
    double hold_time = 0.0;
    double hold_pos_x = 0.0;
    double hold_pos_y = 0.0;
    double hold_pos_z = 0.0;

};



// ---------------- Polynomial evaluation helper ----------------
double evalPoly(const std::vector<double> &coeffs, double t, double T, uint32_t der_order=0)
{
    double val = 0.0;
    size_t n = coeffs.size();
    for (size_t i = der_order; i < n; ++i)
    {
        double term = coeffs[i];
        for (size_t k = 0; k < der_order; ++k)
            term *= (i - k);
        val += term * std::pow(t, i - der_order);
    }
    return val / std::pow(T, der_order);
}

// ---------------- Navigator Node ----------------
class NavigatorNode : public rclcpp::Node 
{
public:
    using FollowTrajectory = drone_interfaces::action::FollowTrajectory;
    using GoalHandleFollowTrajectory = rclcpp_action::ServerGoalHandle<FollowTrajectory>;

    NavigatorNode() : Node("navigator_node")
    {
        // Publisher: MAVROS PositionTarget
        setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("/drone/navigator/setpoint_local", 10);

        // Subscriber: local drone position
        position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/fc_interface/position_local", 10,
            std::bind(&NavigatorNode::positionCallback, this, std::placeholders::_1)
        );

        apriltag_distance_sub_ = this->create_subscription<drone_interfaces::msg::ApriltagPosition>(
            "/drone/apriltag_guidance/apriltag_distance", 10,
            std::bind(&NavigatorNode::apriltagCallback, this, std::placeholders::_1)
        );

        detector_status_sub = this->create_subscription<std_msgs::msg::Bool>(
            "/drone/camera_calibration/detector_status", 10,
            std::bind(&NavigatorNode::detectorStatusCallback, this, std::placeholders::_1)
        );


        // Service client
        takeoff_client_ = this->create_client<drone_interfaces::srv::Takeoff>("/drone/state_machine/takeoff_srv");
        land_client_ = this->create_client<drone_interfaces::srv::Land>("/drone/state_machine/land_srv");
        trajectory_client_ = this->create_client<drone_interfaces::srv::GenerateGlobalTrajectory>("/drone/global_trajectory_planner/generate_global_trajectory");
        activate_detector_client_ = this->create_client<drone_interfaces::srv::ActivateDetector>("drone/camera_calibration/activate_detector");


        // Action server: follow trajectory
        action_server_ = rclcpp_action::create_server<FollowTrajectory>(
            this,
            "drone/navigator/follow_trajectory",
            std::bind(&NavigatorNode::handleGoal, this, std::placeholders::_1),
            std::bind(&NavigatorNode::handleCancel, this, std::placeholders::_1),
            std::bind(&NavigatorNode::handleAccepted, this, std::placeholders::_1)
        );

    }

private:
    // ---------------- ROS interfaces ----------------
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<drone_interfaces::msg::ApriltagPosition>::SharedPtr apriltag_distance_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detector_status_sub;
    rclcpp_action::Server<FollowTrajectory>::SharedPtr action_server_;
    rclcpp::Client<drone_interfaces::srv::GenerateGlobalTrajectory>::SharedPtr trajectory_client_;
    rclcpp::Client<drone_interfaces::srv::Takeoff>::SharedPtr takeoff_client_;
    rclcpp::Client<drone_interfaces::srv::Land>::SharedPtr land_client_;
    rclcpp::Client<drone_interfaces::srv::ActivateDetector>::SharedPtr activate_detector_client_;

    // ---------------- Trajectory storage ----------------
    std::vector<TrajectorySegment> trajectory_;
    double total_trajectory_time_ = 0.0;

    size_t current_major_segment_ = 0;
    size_t current_minor_segment_ = 0; 
    bool holding_ = false;
    double deviation_;

    rclcpp::Time start_minor_time_;
    rclcpp::Time start_holding_time_;
    rclcpp::Time start_time_;
    
    std::vector<double> minor_coeffs_x;
    std::vector<double> minor_coeffs_y;
    std::vector<double> minor_coeffs_z;

    // --------------- Trajectory Generation ----------------------
    mavros_msgs::msg::PositionTarget setpoint;

    double vmax = 2.0;                                          // Maximum allowed velocity
    double amax = 2.0;                                          // Maximum allowed acceleration
    int order = 5;                                              // Polynomial order (optional)
    int continuity_order = 3;                                   // Continuity constraints (optional)
    int min_derivative_order = 2;                               // Min derivative for QP (optional)


    // ---------------- Received Waypoints  ----------------
    std::vector<geometry_msgs::msg::Point> waypoints_;
    std::vector<double> hold_times_;
    std::vector<uint32_t> hold_indexes_;

    // settings
    double position_tolerance_ = 1;                           // Position tolerance for stopping


    // setting up timer
    rclcpp::TimerBase::SharedPtr trajectory_timer_;
    std::shared_ptr<GoalHandleFollowTrajectory> active_goal_;


    // ---------------- Drone pose ----------------
    geometry_msgs::msg::PoseStamped current_pose_;


    // action features
    std::shared_ptr<FollowTrajectory::Feedback> feedback_;
    std::shared_ptr<FollowTrajectory::Result> action_result_;

    
    // ---------------- TAKEOFF and LANDING ------------
    bool takeoff_requested_ = false;
    bool land_requested_ = false;
    double land_altitude_ = 0.0; 
    rclcpp::Time takeoff_time_start_;
    rclcpp::Time land_time_start_;
    double landing_threshold_ = 0.75;
    double landing_velocity_ = 0.3;
    double landing_yaw_rate_ = 0.5;

    bool drone_stopped_ = false;

    // ----------------- HOLD ---------------------
    rclcpp::Time hold_time_start_;


    // ----------------- CAMERA ------------------
    bool use_camera_ = false;
    double horizontal_velocity_ = 0.2;
    drone_interfaces::msg::ApriltagPosition apriltag_distance_;
    double apriltag_threshold_ = 0.4; 
    double k_ = 0.05;
    bool detector_status_ = false;
    bool detector_activated_ = false;
    bool distance_received_ = false;


    // ---------------- Drone position callback ----------------
    void positionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {   
        current_pose_ = *msg;
    }

    // --------------- Apriltag Callback ------------------
    void apriltagCallback(const drone_interfaces::msg::ApriltagPosition::SharedPtr msg) {
        apriltag_distance_ = *msg;
        distance_received_ = true;
    }

    void detectorStatusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        detector_status_ = msg->data;
    }


    // ---------------- Action callbacks ----------------
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid)
    {
        if (active_goal_) {
            RCLCPP_WARN(this->get_logger(), "A trajectory is already being executed. Rejecting new goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }


        RCLCPP_INFO(this->get_logger(), "Received new trajectory goal");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {

        (void)goal_handle;
       
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {   

        active_goal_ = goal_handle;
        
        const auto goal = active_goal_->get_goal();
        
        waypoints_ = goal->waypoints;
        hold_times_ = goal->hold_times;
        hold_indexes_ = goal->hold_indexes;  
        use_camera_ = goal->use_camera;

        feedback_ = std::make_shared<FollowTrajectory::Feedback>();
        action_result_ = std::make_shared<FollowTrajectory::Result>();

        // print in console waypoints, hold times and hold indexes
        //RCLCPP_INFO(this->get_logger(), "Waypoints received:");
        //for (size_t i = 0; i < waypoints_.size(); ++i) {
        //     RCLCPP_INFO(this->get_logger(), "Waypoint %zu: (%.2f, %.2f, %.2f), Hold time: %.2f s", 
        //                 i, waypoints_[i].x, waypoints_[i].y, waypoints_[i].z, hold_times_[i]);
        // }


        if (!trajectory_client_->wait_for_service(2s)) {
            
            requestAbortLand("Trajectory generation service not available");
            return;
        }

        generate_trajectory_segment(0);
        
    }


    void generate_trajectory_segment(size_t segment_index) {

        if (segment_index >= hold_indexes_.size() - 1) {
            // All segments done
            RCLCPP_INFO(this->get_logger(), "All trajectory segments generated");
            initialize_trajectory();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generating trajectory segment %zu", segment_index);
        TrajectorySegment seg;


        if (hold_indexes_[segment_index+1] - hold_indexes_[segment_index] == 1) {
            if (waypoints_[hold_indexes_[segment_index]].z < 0.1 && waypoints_[hold_indexes_[segment_index+1]].z > 0.1) {
                // segment type: Takeoff
                seg.type = SegmentType::Takeoff;
                seg.takeoff_altitude = waypoints_[hold_indexes_[segment_index+1]].z;
                trajectory_.push_back(seg);
                
                TrajectorySegment seg_hold;
                if (hold_times_[segment_index+1] > 0.0) {
                    seg_hold.type = SegmentType::Hold;
                    seg_hold.hold_time = this->hold_times_[segment_index+1];
                    seg_hold.hold_pos_x = this->waypoints_[segment_index+1].x;
                    seg_hold.hold_pos_y = this->waypoints_[segment_index+1].y;
                    seg_hold.hold_pos_z = this->waypoints_[segment_index+1].z;
                    trajectory_.push_back(seg_hold);
                }

                generate_trajectory_segment(segment_index+1);
                return;
            } 
            else if (waypoints_[hold_indexes_[segment_index]].z > 0.1 && waypoints_[hold_indexes_[segment_index+1]].z < 0.1) {
                //segment type: land
                seg.type = SegmentType::Land;
                trajectory_.push_back(seg);
                generate_trajectory_segment(segment_index+1);

                TrajectorySegment seg_hold;
                if (hold_times_[segment_index+1] > 0.0) {
                    seg_hold.type = SegmentType::Hold;
                    seg_hold.hold_time = this->hold_times_[segment_index+1];
                    seg_hold.hold_pos_x = this->waypoints_[segment_index+1].x;
                    seg_hold.hold_pos_y = this->waypoints_[segment_index+1].y;
                    seg_hold.hold_pos_z = this->waypoints_[segment_index+1].z;
                    trajectory_.push_back(seg_hold);
                }
                return;
            }
        }
            
        // Prepare waypoints for this segment if it's of type fly
        std::vector<geometry_msgs::msg::Point> segment_waypoints;
        for (size_t j = hold_indexes_[segment_index]; j <= hold_indexes_[segment_index + 1]; ++j) {
            segment_waypoints.push_back(waypoints_[j]);
        }

        auto request = std::make_shared<drone_interfaces::srv::GenerateGlobalTrajectory::Request>();
        request->waypoints = segment_waypoints;
        request->vmax = vmax;
        request->amax = amax;
        request->order = order;
        request->continuity_order = continuity_order;
        request->min_derivative_order = min_derivative_order;


        using ResponseFuture =
        std::shared_future<std::shared_ptr<drone_interfaces::srv::GenerateGlobalTrajectory::Response>>;


        auto result_future = trajectory_client_->async_send_request(
            request,
            [this, segment_index, seg](ResponseFuture future)
            {
                auto result = future.get(); 

                if (!result->success) {
                    RCLCPP_ERROR(this->get_logger(), "Segment %zu generation failed: %s",
                                segment_index, result->message.c_str());
                    requestAbortLand("Trajectory generation failed: " + result->message);
                    return;
                }
                
                TrajectorySegment seg_fly;

                // Save segment
                seg_fly.type = SegmentType::Fly;
                seg_fly.coeffs_x = result->coeffs_x;
                seg_fly.coeffs_y = result->coeffs_y;
                seg_fly.coeffs_z = result->coeffs_z;
                seg_fly.num_segments = result->num_segments;
                seg_fly.num_coeffs_segment = result->num_coeffs_segment;
                seg_fly.periods = result->periods;
                
                this->trajectory_.push_back(seg_fly);
                
                TrajectorySegment seg_hold;

                if (this->hold_times_[segment_index+1] > 0.0) {
                    seg_hold.type = SegmentType::Hold;
                    seg_hold.hold_time = this->hold_times_[segment_index+1];
                    seg_hold.hold_pos_x = this->waypoints_[segment_index+1].x;
                    seg_hold.hold_pos_y = this->waypoints_[segment_index+1].y;
                    seg_hold.hold_pos_z = this->waypoints_[segment_index+1].z;
                    this->trajectory_.push_back(seg_hold);
                }

                // Call next segment
                this->generate_trajectory_segment(segment_index + 1);
                return;
            });
    }



    void initialize_trajectory() {
        RCLCPP_INFO(this->get_logger(), "Initializing trajectory");

        for (size_t i = 0; i < trajectory_.size(); ++i) {
            for (size_t j = 0; j < trajectory_[i].num_segments; ++j) {
                total_trajectory_time_ += trajectory_[i].periods[j];
            }
        }
    
        // typemask for setpoints
        setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        


        RCLCPP_INFO(this->get_logger(), "Executing major segment: %zu, minor segment: %zu", current_major_segment_, current_minor_segment_);

        start_minor_time_ = this->now();
        start_time_ = this->now();

        if (trajectory_timer_) {
            trajectory_timer_->cancel();
            trajectory_timer_.reset();
        }    
    
        trajectory_timer_ = this->create_wall_timer(
            20ms, std::bind(&NavigatorNode::trajectoryControlLoop, this)
        );
    }

    void requestAbortLand(std::string reason = "", bool canceled = false) {
        if (!land_requested_) {
            RCLCPP_INFO(this->get_logger(), "Requesting Land");
            land_requested_ = true;
            auto request = std::make_shared<drone_interfaces::srv::Land::Request>();
            auto result_future = land_client_->async_send_request(request);
        } else {
            if (current_pose_.pose.position.z < 0.1) {
                land_requested_ = false;
                action_result_->success = false;
                action_result_->message = reason;
                if (canceled)
                    active_goal_->canceled(action_result_);
                else
                    active_goal_->abort(action_result_);
                cleanupGoal();
                return;        
            } else {
                return;
            }
        }
    }

   
    void trajectoryControlLoop()
    {      
        if (!active_goal_) return;

        // Check for cancel request
        if (active_goal_->is_canceling()) {
            if (!drone_stopped_) stop_drone("Goal canceled by client");

            requestAbortLand("Trajectory aborted by client", true);
            return;
        }


        if (trajectory_.empty()) return;

        if (current_major_segment_ >= trajectory_.size()) {
            RCLCPP_INFO(this->get_logger(), "Trajectory completed");
            stop_drone("Trajectory completed");
            action_result_->success = true;
            action_result_->message = "Trajectory completed successfully.";
            active_goal_->succeed(action_result_);
            cleanupGoal();
            return;
        }


        TrajectorySegment seg = trajectory_[current_major_segment_];

        minor_coeffs_x.clear();
        minor_coeffs_y.clear();
        minor_coeffs_z.clear();

        for (size_t i = 0; i < seg.num_coeffs_segment; ++i) {
            auto current_index = i + seg.num_coeffs_segment*current_minor_segment_;
            minor_coeffs_x.push_back(seg.coeffs_x[current_index]);
            minor_coeffs_y.push_back(seg.coeffs_y[current_index]);
            minor_coeffs_z.push_back(seg.coeffs_z[current_index]);
        }

        setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        setpoint.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_YAW;
        

        switch (seg.type) {
            // ------------------- TAKEOFF ----------------
            case SegmentType::Takeoff: 
                if (!takeoff_requested_) {
                    RCLCPP_INFO(this->get_logger(), "Requesting takeoff to altitude: %.2f m", seg.takeoff_altitude);
                    auto request = std::make_shared<drone_interfaces::srv::Takeoff::Request>();
                    request->altitude = seg.takeoff_altitude;
                    takeoff_time_start_ = this->now();
                    takeoff_requested_ = true;
                    auto result_future = takeoff_client_->async_send_request(request);
                } else {
                    if (abs(current_pose_.pose.position.z - seg.takeoff_altitude) < 0.4) {
                        takeoff_requested_ = false;
                        ++current_major_segment_;
                        current_minor_segment_ = 0;
                        start_minor_time_ = this->now();
                        total_trajectory_time_ += (this->now() - takeoff_time_start_).seconds();
                    }
                }

                feedback_->setpoint.x = current_pose_.pose.position.x;
                feedback_->setpoint.y = current_pose_.pose.position.y;
                feedback_->setpoint.z = seg.takeoff_altitude;
                feedback_->local_progress = current_pose_.pose.position.z/seg.takeoff_altitude;
                feedback_->deviation = abs(seg.takeoff_altitude-current_pose_.pose.position.z);

                break;
            
            // ------------------- LAND ----------------
            case SegmentType::Land:

                
                
                if (use_camera_ && !detector_status_ && !detector_activated_) {
                    auto request = std::make_shared<drone_interfaces::srv::ActivateDetector::Request>();
                    request->activate = true;
                    activate_detector_client_->async_send_request(request);
                    detector_activated_ = true;
                }

                setpoint.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                                        mavros_msgs::msg::PositionTarget::IGNORE_AFZ |                            
                                        mavros_msgs::msg::PositionTarget::IGNORE_YAW;
                
                if (current_pose_.pose.position.z >= landing_threshold_) {
                    
                    double vx = 0.0;
                    double vy = 0.0;
                    double vz = -landing_velocity_;
                    double yaw_rate = 0.0;


                    if (use_camera_ && detector_status_) {

                        if (!distance_received_) {
                            RCLCPP_WARN(this->get_logger(), "No apriltag distance received yet, descending...");
                            vz = 0.0;
                            vx = 0.0;
                            vz = -landing_velocity_;
                            yaw_rate = landing_yaw_rate_;

                        } else {

                            if (!apriltag_distance_.timeout_expired) {
                                vx = k_*apriltag_distance_.center.x;
                                vy = -k_*apriltag_distance_.center.y;

                                vx = std::clamp(vx, -horizontal_velocity_, horizontal_velocity_);
                                vy = std::clamp(vy, -horizontal_velocity_, horizontal_velocity_);
                                
                                yaw_rate = 0.0;
                                bool centered = abs(apriltag_distance_.center.x)<=apriltag_threshold_ && abs(apriltag_distance_.center.y)<=apriltag_threshold_;
                                
                                if (centered) {
                                    vz = -landing_velocity_;
                                    
                                } else {
                                    vz = 0.0;                                    
                                }

                            } else {
                                vx = 0.0;
                                vy = 0.0;
                                vz = -landing_velocity_;
                                yaw_rate = landing_yaw_rate_;
                            }            
                        }
                    }
                    
                    setpoint.velocity.x = vx;
                    setpoint.velocity.y = vy;
                    setpoint.velocity.z = vz;
                    setpoint.yaw_rate = yaw_rate;

                    setpoint.header.stamp = this->now();
                    
                
                } else {

                    setpoint.velocity.x = 0.0;
                    setpoint.velocity.y = 0.0;
                    setpoint.velocity.z = 0.0;
                    setpoint.yaw_rate = 0.0;
                    setpoint.header.stamp = this->now();


                    if (!land_requested_) {
                        RCLCPP_INFO(this->get_logger(), "Requesting Land");
                        auto request = std::make_shared<drone_interfaces::srv::Land::Request>();
                        auto result_future = land_client_->async_send_request(request);
                        land_altitude_ = current_pose_.pose.position.z;
                        land_time_start_ = this->now();
                        land_requested_ = true;
                    } else {
                        if (current_pose_.pose.position.z < 0.1) {
                            land_requested_ = false;
                            ++current_major_segment_;
                            current_minor_segment_ = 0;
                            start_minor_time_ = this->now();
                            total_trajectory_time_ += (this->now() - land_time_start_).seconds();
                        }
                    }
                }

                setpoint_pub_->publish(setpoint);
                feedback_->setpoint.x = current_pose_.pose.position.x;
                feedback_->setpoint.y = current_pose_.pose.position.y;
                feedback_->setpoint.z = 0.0;
                feedback_->local_progress = current_pose_.pose.position.z/land_altitude_;
                feedback_->deviation = abs(current_pose_.pose.position.z);
                
                break;
            
            // ------------------- HOLD ----------------
            case SegmentType::Hold: 
                if (!holding_) {
                    hold_time_start_ = this->now();
                    holding_ = true;
                }

                if (seg.hold_time > (this->now() - hold_time_start_).seconds()) {                    
                    setpoint.position.x = seg.hold_pos_x;
                    setpoint.position.y = seg.hold_pos_y;
                    setpoint.position.z = seg.hold_pos_z;

                    setpoint.velocity.x = 0.0;
                    setpoint.velocity.y = 0.0;
                    setpoint.velocity.z = 0.0;

                    setpoint.acceleration_or_force.x = 0.0;
                    setpoint.acceleration_or_force.y = 0.0;
                    setpoint.acceleration_or_force.z = 0.0;

                    setpoint.yaw_rate = 0.0;

                    setpoint.header.stamp = this->now();
                    setpoint_pub_->publish(setpoint);

                } else {
                    current_minor_segment_ = 0;
                    ++current_major_segment_;
                    start_minor_time_ = this->now();
                    holding_ = false;
                    total_trajectory_time_ += seg.hold_time;
                }

                // compute deviation from actual trajectory
                deviation_ = sqrt(pow(setpoint.position.x - current_pose_.pose.position.x, 2) +
                            pow(setpoint.position.y - current_pose_.pose.position.y, 2) +
                            pow(setpoint.position.z - current_pose_.pose.position.z, 2));

                // Publish feedback
                feedback_->setpoint.x = setpoint.position.x;
                feedback_->setpoint.y = setpoint.position.y;
                feedback_->setpoint.z = setpoint.position.z;
                feedback_->local_progress = (this->now() - hold_time_start_).seconds()/seg.hold_time;
                feedback_->deviation = deviation_;

                break;
            // ------------------- FLY ----------------------
            case SegmentType::Fly:

                double tau = ((this->now() - start_minor_time_).seconds())/seg.periods[current_minor_segment_];

                // setpoint generation
                setpoint.header.stamp = this->now();

                // publish setpoints
                auto current_period = seg.periods[current_minor_segment_];
               

                setpoint.position.x = evalPoly(minor_coeffs_x, tau, current_period, 0);
                setpoint.position.y = evalPoly(minor_coeffs_y, tau, current_period, 0);
                setpoint.position.z = evalPoly(minor_coeffs_z, tau, current_period, 0);

                setpoint.velocity.x = evalPoly(minor_coeffs_x, tau, current_period, 1);
                setpoint.velocity.y = evalPoly(minor_coeffs_y, tau, current_period, 1);
                setpoint.velocity.z = evalPoly(minor_coeffs_z, tau, current_period, 1);

                setpoint.acceleration_or_force.x = evalPoly(minor_coeffs_x, tau, current_period, 2);
                setpoint.acceleration_or_force.y = evalPoly(minor_coeffs_y, tau, current_period, 2);
                setpoint.acceleration_or_force.z = evalPoly(minor_coeffs_z, tau, current_period, 2);

                setpoint.yaw_rate = 0.0;

                setpoint.header.stamp = this->now();
                setpoint_pub_->publish(setpoint);
           

                // compute deviation from actual trajectory
                deviation_ = sqrt(pow(setpoint.position.x - current_pose_.pose.position.x, 2) +
                            pow(setpoint.position.y - current_pose_.pose.position.y, 2) +
                            pow(setpoint.position.z - current_pose_.pose.position.z, 2));

                // computer total trajectory progress
                auto global_progress = (this->now() - start_time_).seconds() / total_trajectory_time_;

                // Publish feedback
                feedback_->setpoint.x = setpoint.position.x;
                feedback_->setpoint.y = setpoint.position.y;
                feedback_->setpoint.z = setpoint.position.z;
                feedback_->local_progress = tau;
                feedback_->global_progress = global_progress;
                feedback_->deviation = deviation_;

                if (tau >= 1.0) {
                    start_minor_time_ = this->now();
                    if (current_minor_segment_ + 1>= seg.num_segments) {
                        ++current_major_segment_;
                        current_minor_segment_ = 0;
                    } else {
                        ++current_minor_segment_;
                    }
                }


                if (deviation_ > position_tolerance_ && !drone_stopped_) {
                    stop_drone("Too large deviation from planned path: " + std::to_string(deviation_) + " m");

                    requestAbortLand("Trajectory aborted because of too large deviation from planned path.");
                }

                break;
        } 

        double global_progress; 

        // computer total trajectory progress
        if (total_trajectory_time_>0.0) {
            global_progress = (this->now() - start_time_).seconds() / total_trajectory_time_;
        } else {
            global_progress = 0.0;
        }
        
        feedback_->segment_type = toString(seg.type);
        feedback_->current_point.x = current_pose_.pose.position.x;
        feedback_->current_point.y = current_pose_.pose.position.y;
        feedback_->current_point.z = current_pose_.pose.position.z;
        feedback_->current_major_segment = current_major_segment_;
        feedback_->current_minor_segment = current_minor_segment_;
        feedback_->global_progress = global_progress;

        active_goal_->publish_feedback(feedback_);
    }

    // ---------------- Stop drone safely ----------------
    void stop_drone(std::string reason = "")
    {
        drone_stopped_ = true;

        RCLCPP_WARN(this->get_logger(), "Stopping drone! Reason: %s", reason.c_str());
        mavros_msgs::msg::PositionTarget stop_point;
        stop_point.header.stamp = this->now();
        stop_point.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        stop_point.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
                            mavros_msgs::msg::PositionTarget::IGNORE_PY |
                            mavros_msgs::msg::PositionTarget::IGNORE_PZ |
                            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
                            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
                            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::msg::PositionTarget::IGNORE_YAW |
                            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        stop_point.velocity.x = 0.0;
        stop_point.velocity.y = 0.0;
        stop_point.velocity.z = 0.0;
            
        setpoint_pub_->publish(stop_point);

    }


    void cleanupGoal() {
    if (trajectory_timer_) {
        trajectory_timer_->cancel();
        trajectory_timer_.reset();
    }
    active_goal_.reset();
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


