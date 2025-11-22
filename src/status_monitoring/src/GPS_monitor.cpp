#include "status_monitoring/GPS_monitor.hpp"

GPSMonitor::GPSMonitor(rclcpp::Node* node, double timeout_threshold)
: BaseMonitor(node), timeout_threshold_(timeout_threshold),
  stale_(true), nominal_(false), no_fix_(true), no_service_(true), accuracy_unknown_(true)
{
    fix_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/drone/fc_interface/fix", 10,
        std::bind(&GPSMonitor::fix_callback, this, std::placeholders::_1)
    );
}

void GPSMonitor::fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    nav_sat_fix_ = *msg;
    last_update_ = node_->now();
    stale_ = is_stale();
}

bool GPSMonitor::is_stale() const {
    if (!last_update_.nanoseconds()) return true;
    double age = (node_->now().nanoseconds() - last_update_.nanoseconds()) / 1e9;
    return age > timeout_threshold_;
}

std::map<std::string, bool> GPSMonitor::update() {
    no_fix_ = nav_sat_fix_.status.status == -1;
    no_service_ = (nav_sat_fix_.status.service & 1) == 0;
    accuracy_unknown_ = nav_sat_fix_.position_covariance_type == 0;

    nominal_ = !(no_fix_ || no_service_ || accuracy_unknown_ || stale_);

    return {
        {"status", nominal_},
        {"GPS_no_fix", no_fix_},
        {"GPS_no_service", no_service_},
        {"GPS_accuracy_unknown", accuracy_unknown_},
        {"GPS_stale", stale_}
    };
}
