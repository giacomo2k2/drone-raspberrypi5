#pragma once

#include "base_monitor.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <map>
#include <string>

class GPSMonitor : public BaseMonitor {
public:
    explicit GPSMonitor(rclcpp::Node* node, double timeout_threshold = 1.0);

    void fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    bool is_stale() const override;
    std::map<std::string, bool> update() override;

private:
    double timeout_threshold_;
    sensor_msgs::msg::NavSatFix nav_sat_fix_;

    rclcpp::Time last_update_;
    bool stale_;
    bool nominal_;
    bool no_fix_;
    bool no_service_;
    bool accuracy_unknown_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
};
