#include "status_monitoring/estimator_monitor.hpp"

EstimatorMonitor::EstimatorMonitor(rclcpp::Node* node, double timeout_threshold)
: BaseMonitor(node), timeout_threshold_(timeout_threshold),
  stale_pos_(true), stale_vel_(true), nominal_(false)
{
    local_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/drone/fc_interface/position_local", 10,
        std::bind(&EstimatorMonitor::local_position_cb, this, std::placeholders::_1)
    );

    local_velocity_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/drone/fc_interface/velocity_local", 10,
        std::bind(&EstimatorMonitor::local_velocity_cb, this, std::placeholders::_1)
    );
}

void EstimatorMonitor::local_position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    local_position_ = *msg;
    last_update_pos_ = node_->now();
    stale_pos_ = is_stale_pos();
}

void EstimatorMonitor::local_velocity_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    local_velocity_ = *msg;
    last_update_vel_ = node_->now();
    stale_vel_ = is_stale_vel();
}

bool EstimatorMonitor::is_stale_pos() const {
    if (!last_update_pos_.nanoseconds()) return true;
    double age = (node_->now().nanoseconds() - last_update_pos_.nanoseconds()) / 1e9;
    return age > timeout_threshold_;
}

bool EstimatorMonitor::is_stale_vel() const {
    if (!last_update_vel_.nanoseconds()) return true;
    double age = (node_->now().nanoseconds() - last_update_vel_.nanoseconds()) / 1e9;
    return age > timeout_threshold_;
}

std::map<std::string, bool> EstimatorMonitor::update() {
    nominal_ = !stale_pos_ && !stale_vel_; // assumes messages received

    return {
        {"status", nominal_},
        {"no_position", stale_pos_},  // True if position is stale or missing
        {"no_velocity", stale_vel_}   // True if velocity is stale or missing
    };
}
