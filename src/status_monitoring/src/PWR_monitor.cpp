#include "status_monitoring/PWR_monitor.hpp"

PWRMonitor::PWRMonitor(rclcpp::Node* node,
                       double min_voltage,
                       double max_voltage,
                       double max_current,
                       double timeout_threshold)
: BaseMonitor(node),
  min_voltage_(min_voltage), max_voltage_(max_voltage), max_current_(max_current),
  timeout_threshold_(timeout_threshold),
  stale_(true), nominal_(false), overvoltage_(false), undervoltage_(true), overcurrent_(false)
{
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/drone/fc_interface/battery", 10,
        std::bind(&PWRMonitor::battery_callback, this, std::placeholders::_1)
    );
}

void PWRMonitor::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    battery_state_ = *msg;
    last_update_ = node_->now();
    stale_ = is_stale();
}

bool PWRMonitor::is_stale() const {
    if (!last_update_.nanoseconds()) return true;
    double age = (node_->now().nanoseconds() - last_update_.nanoseconds()) / 1e9;
    return age > timeout_threshold_;
}

std::map<std::string, bool> PWRMonitor::update() {
    overvoltage_ = battery_state_.voltage > max_voltage_;
    undervoltage_ = battery_state_.voltage < min_voltage_;
    overcurrent_ = !std::isnan(battery_state_.current) && battery_state_.current > max_current_;

    // stale_ is already updated in callback
    nominal_ = !(overvoltage_ || undervoltage_ || overcurrent_ || stale_);

    return {
        {"status", nominal_},
        {"PWR_overvoltage", overvoltage_},
        {"PWR_undervoltage", undervoltage_},
        {"PWR_overcurrent", overcurrent_},
        {"PWR_stale", stale_}
    };
}
