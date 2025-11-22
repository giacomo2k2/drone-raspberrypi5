#pragma once

#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>

class BaseMonitor {
public:
    explicit BaseMonitor(rclcpp::Node* node) : node_(node) {}
    virtual ~BaseMonitor() = default;

    // Update function returns a map of status flags
    virtual std::map<std::string, bool> update() = 0;

    // Optional stale check, now virtual and const
    virtual bool is_stale() const { return false; }

protected:
    rclcpp::Node* node_;
};
