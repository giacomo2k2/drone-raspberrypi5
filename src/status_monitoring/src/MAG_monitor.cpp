#include "status_monitoring/MAG_monitor.hpp"

MAGMonitor::MAGMonitor(rclcpp::Node* node, double timeout_threshold)
: BaseMonitor(node), timeout_threshold_(timeout_threshold),
  stale_(true), nominal_(false), covariance_unknown_(true)
{
    mag_sub_ = node_->create_subscription<sensor_msgs::msg::MagneticField>(
        "/drone/fc_interface/mag", 10,
        std::bind(&MAGMonitor::mag_callback, this, std::placeholders::_1)
    );
}

void MAGMonitor::mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    magnetic_field_ = *msg;
    last_update_ = node_->now();
    stale_ = is_stale();
}

bool MAGMonitor::is_stale() const {
    if (!last_update_.nanoseconds()) return true;
    double age = (node_->now().nanoseconds() - last_update_.nanoseconds()) / 1e9;
    return age > timeout_threshold_;
}

std::map<std::string, bool> MAGMonitor::update() {
    covariance_unknown_ = std::all_of(
        magnetic_field_.magnetic_field_covariance.begin(),
        magnetic_field_.magnetic_field_covariance.end(),
        [](double cov){ return cov == 0.0; }
    );

    nominal_ = !(covariance_unknown_ || stale_);

    return {
        {"status", nominal_},
        {"MAG_covariance_unknown", covariance_unknown_},
        {"MAG_stale", stale_}
    };
}
