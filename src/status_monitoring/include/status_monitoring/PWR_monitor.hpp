#pragma once

#include "base_monitor.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include <map>
#include <string>
#include <cmath>

class PWRMonitor : public BaseMonitor {
public:
    explicit PWRMonitor(rclcpp::Node* node, 
                        double min_voltage = 10.5,
                        double max_voltage = 14.1,
                        double max_current = 20.0,
                        double timeout_threshold = 1.0);

    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    bool is_stale() const override;
    std::map<std::string, bool> update() override;

private:
    double min_voltage_;
    double max_voltage_;
    double max_current_;
    double timeout_threshold_;

    sensor_msgs::msg::BatteryState battery_state_;

    rclcpp::Time last_update_;
    bool stale_;
    bool nominal_;
    bool overvoltage_;
    bool undervoltage_;
    bool overcurrent_;

    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
};
