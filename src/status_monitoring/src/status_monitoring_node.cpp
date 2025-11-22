#include <rclcpp/rclcpp.hpp>
#include <map>
#include <memory>
#include "status_monitoring/base_monitor.hpp"
#include "status_monitoring/MAG_monitor.hpp"
#include "status_monitoring/PWR_monitor.hpp"
#include "status_monitoring/GPS_monitor.hpp"
#include "status_monitoring/IMU_monitor.hpp"
#include "status_monitoring/estimator_monitor.hpp"

#include "drone_interfaces/msg/failsafe_event.hpp"
#include "drone_interfaces/msg/pre_arm_check.hpp"
#include "drone_interfaces/msg/components_status.hpp"

class StatusMonitoringNode : public rclcpp::Node {

public:
    StatusMonitoringNode() : Node("status_monitoring") 
    {
        // initialize monitors
        monitors_["PWR"] = std::make_shared<PWRMonitor>(this);
        monitors_["GPS"] = std::make_shared<GPSMonitor>(this);
        monitors_["IMU"] = std::make_shared<IMUMonitor>(this);
        monitors_["MAG"] = std::make_shared<MAGMonitor>(this, 5.0);
        monitors_["ESTIMATOR"] = std::make_shared<EstimatorMonitor>(this);

        // init components_status
        for (auto& [name, monitor] : monitors_) {
            drone_interfaces::msg::SingleComponentStatus scs;
            scs.component_name = name;
            scs.status = "NOMINAL";
            components_status_.components_status.push_back(scs);
        }

        // publishers
        prearm_pub_ = this->create_publisher<drone_interfaces::msg::PreArmCheck>(
            "/drone/status_monitoring/prearm", 10
        );
        components_status_pub_ = this->create_publisher<drone_interfaces::msg::ComponentsStatus>(
            "/drone/status_monitoring/components_status", 10
        );

        // timer
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
            std::bind(&StatusMonitoringNode::monitor_loop_cb, this)
        );
    }

private:
    std::map<std::string, std::shared_ptr<BaseMonitor>> monitors_;
    drone_interfaces::msg::ComponentsStatus components_status_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<drone_interfaces::msg::PreArmCheck>::SharedPtr prearm_pub_;
    rclcpp::Publisher<drone_interfaces::msg::ComponentsStatus>::SharedPtr components_status_pub_;


    void update() {
        for (auto& [name, monitor] : monitors_) {
            auto result = monitor->update();
            // update components_status
            for (auto& comp : components_status_.components_status) {
                if (comp.component_name == name) {
                    comp.status = result["status"] ? "NOMINAL" : "CRITICAL";
                }
            }
        }
    }


    void monitor_loop_cb() {
        update();

        drone_interfaces::msg::PreArmCheck prearm_msg;
        bool checks_passed = true;

        for (auto& [name, monitor] : monitors_) {
            auto result = monitor->update();
            if (!result["status"]) checks_passed = false;
            for (auto& [key, value] : result) {
                if (key != "status" && value) {
                    prearm_msg.failed_components.push_back(key);
                }
            }
        }

        prearm_msg.checks_passed = checks_passed;
        auto now = this->get_clock()->now();
        prearm_msg.timestamp = now;
        components_status_.timestamp = now;
        prearm_msg.header.stamp = now;
        components_status_.header.stamp = now;

        prearm_pub_->publish(prearm_msg);
        components_status_pub_->publish(components_status_);
    }   
};
    

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatusMonitoringNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}