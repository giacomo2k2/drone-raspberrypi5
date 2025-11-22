#pragma once

#include "base_monitor.hpp"
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <map>
#include <string>
#include <algorithm>

class MAGMonitor : public BaseMonitor {
public:
    explicit MAGMonitor(rclcpp::Node* node, double timeout_threshold = 1.0);

    void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
    bool is_stale() const override;
    std::map<std::string, bool> update() override;

private:
    double timeout_threshold_;
    sensor_msgs::msg::MagneticField magnetic_field_;

    rclcpp::Time last_update_;
    bool stale_;
    bool nominal_;
    bool covariance_unknown_;

    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
};
