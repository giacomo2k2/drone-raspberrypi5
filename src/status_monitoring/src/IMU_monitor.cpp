#include "status_monitoring/IMU_monitor.hpp"

IMUMonitor::IMUMonitor(rclcpp::Node* node, double timeout_threshold)
: BaseMonitor(node), timeout_threshold_(timeout_threshold),
  stale_(true), nominal_(false), covariance_unknown_(true),
  orientation_unknown_(true), angular_velocity_unknown_(true), linear_acceleration_unknown_(true)
{
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/drone/fc_interface/imu", 10,
        std::bind(&IMUMonitor::imu_callback, this, std::placeholders::_1)
    );
}

void IMUMonitor::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_ = *msg;
    last_update_ = node_->now();
    stale_ = is_stale();
}

bool IMUMonitor::is_stale() const {
    if (!last_update_.nanoseconds()) return true;
    double age = (node_->now().nanoseconds() - last_update_.nanoseconds()) / 1e9;
    return age > timeout_threshold_;
}

std::map<std::string, bool> IMUMonitor::update() {
    // check if all covariances are zero
    covariance_unknown_ = std::all_of(imu_.orientation_covariance.begin(), imu_.orientation_covariance.end(),
                                      [](double c){ return c == 0.0; }) ||
                          std::all_of(imu_.angular_velocity_covariance.begin(), imu_.angular_velocity_covariance.end(),
                                      [](double c){ return c == 0.0; }) ||
                          std::all_of(imu_.linear_acceleration_covariance.begin(), imu_.linear_acceleration_covariance.end(),
                                      [](double c){ return c == 0.0; });

    orientation_unknown_ = imu_.orientation_covariance[0] == -1.0;
    angular_velocity_unknown_ = imu_.angular_velocity_covariance[0] == -1.0;
    linear_acceleration_unknown_ = imu_.linear_acceleration_covariance[0] == -1.0;

    nominal_ = !(covariance_unknown_ || orientation_unknown_ || angular_velocity_unknown_ || linear_acceleration_unknown_ || stale_);

    return {
        {"status", nominal_},
        {"IMU_covariance_unknown", covariance_unknown_},
        {"IMU_orientation_unknown", orientation_unknown_},
        {"IMU_angular_velocity_unknown", angular_velocity_unknown_},
        {"IMU_linear_acceleration_unknown", linear_acceleration_unknown_},
        {"IMU_stale", stale_}
    };
}
