#pragma once

#include "base_monitor.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <map>
#include <string>
#include <algorithm>

class IMUMonitor : public BaseMonitor {
public:
    explicit IMUMonitor(rclcpp::Node* node, double timeout_threshold = 1.0);

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    bool is_stale() const override;
    std::map<std::string, bool> update() override;

private:
    double timeout_threshold_;
    sensor_msgs::msg::Imu imu_;

    rclcpp::Time last_update_;
    bool stale_;
    bool nominal_;
    bool covariance_unknown_;
    bool orientation_unknown_;
    bool angular_velocity_unknown_;
    bool linear_acceleration_unknown_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};
