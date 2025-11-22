#pragma once

#include "base_monitor.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <map>
#include <string>

class EstimatorMonitor : public BaseMonitor {
public:
    explicit EstimatorMonitor(rclcpp::Node* node, double timeout_threshold = 1.0);

    void local_position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void local_velocity_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    bool is_stale_pos() const;
    bool is_stale_vel() const;
    std::map<std::string, bool> update() override;

private:
    double timeout_threshold_;

    geometry_msgs::msg::PoseStamped local_position_;
    geometry_msgs::msg::TwistStamped local_velocity_;

    rclcpp::Time last_update_pos_;
    rclcpp::Time last_update_vel_;

    bool stale_pos_;
    bool stale_vel_;
    bool nominal_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_velocity_sub_;
};
