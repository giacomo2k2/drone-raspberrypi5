#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

class FCInterfaceNode : public rclcpp::Node {
public: 
    FCInterfaceNode() : Node("fc_interface") {
        
    
        /*Publishers*/

        // Publish a simplified state for other nodes
        drone_state_pub = this->create_publisher<mavros_msgs::msg::State>(
            "/drone/fc_interface/state", 10
        );

        battery_pub = this->create_publisher<sensor_msgs::msg::BatteryState>(
            "/drone/fc_interface/battery", 10
        );

        fix_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/drone/fc_interface/fix", 10
        );

        imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(
            "/drone/fc_interface/imu", 10
        );

        mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(
            "/drone/fc_interface/mag", 10
        );

        position_local_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/drone/fc_interface/position_local", 10
        );

        position_global_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/drone/fc_interface/position_global", 10
        );

        velocity_local_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/drone/fc_interface/velocity_local", 10
        );

        heading_pub = this->create_publisher<std_msgs::msg::Float64>(
            "/drone/fc_interface/heading", 10
        );


        rc_pub = this->create_publisher<mavros_msgs::msg::RCIn>(
            "/drone/fc_interface/rc", 10
        );

        rclcpp::QoS qos_state_origin(rclcpp::KeepLast(10));
        qos_state_origin.reliable();
        qos_state_origin.transient_local();

        origin_global_pub = this->create_publisher<geographic_msgs::msg::GeoPointStamped>(
            "/drone/fc_interface/global_origin", qos_state_origin
        );

        // safety_polygon_pub = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
        //     "/mavros/safety_area/set",  qos_state_origin
        // );

        rclcpp::QoS qos_setpoint(rclcpp::KeepLast(10));
        qos_setpoint.best_effort();

        setpoint_local_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", qos_setpoint
        );


        /*Subscribers*/
        rclcpp::QoS qos_rc(rclcpp::KeepLast(10));
        qos_rc.reliable();


        rc_sub = this->create_subscription<mavros_msgs::msg::RCIn>(
            "/mavros/rc/in", qos_rc,
            std::bind(&FCInterfaceNode::rcCallback, this, std::placeholders::_1)
        );

        rclcpp::QoS qos_state(rclcpp::KeepLast(10));
        qos_state.reliable();
        qos_state.transient_local();

        // Subscribe to MAVROS state
        state_sub = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_state, 
            std::bind(&FCInterfaceNode::stateCallback, this, std::placeholders::_1)
        );

        origin_global_sub = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
            "/mavros/global_position/gp_origin", qos_state,
            std::bind(&FCInterfaceNode::originGlobalCallback, this, std::placeholders::_1)
        );

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();

        battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", qos, 
            std::bind(&FCInterfaceNode::batteryCallback, this, std::placeholders::_1)
        );
        
        fix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/raw/fix", qos, 
            std::bind(&FCInterfaceNode::fixCallback, this, std::placeholders::_1)
        );

        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", qos, 
            std::bind(&FCInterfaceNode::imuCallback, this, std::placeholders::_1)
        );

        mag_sub = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "/mavros/imu/mag", qos, 
            std::bind(&FCInterfaceNode::magCallback, this, std::placeholders::_1)
        );

        setpoint_local_sub = this->create_subscription<mavros_msgs::msg::PositionTarget>(
            "/drone/navigator/setpoint_local", qos,
            std::bind(&FCInterfaceNode::setpointLocalCallback, this, std::placeholders::_1)
        );

        position_local_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos,
            std::bind(&FCInterfaceNode::positionLocalCallback, this, std::placeholders::_1)
        );

        position_global_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global", qos,
            std::bind(&FCInterfaceNode::positionGlobalCallback, this, std::placeholders::_1)
        );

        velocity_local_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_local", qos,
            std::bind(&FCInterfaceNode::velocityLocalCallback, this, std::placeholders::_1)
        );
        
        heading_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/compass_hdg", qos,
            std::bind(&FCInterfaceNode::headingCallback, this, std::placeholders::_1)
        );

    }

private: 

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg) {
        // forward the state to a "clean" topic
        drone_state_pub->publish(*msg);
    }

    void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
        battery_pub->publish(*msg);
    }

    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        fix_pub->publish(*msg);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_pub->publish(*msg);
    }

    void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        mag_pub->publish(*msg);
    }

    void setpointLocalCallback(const mavros_msgs::msg::PositionTarget::SharedPtr msg) {
        setpoint_local_pub->publish(*msg);
    }

    void positionLocalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        position_local_pub->publish(*msg);
    }

    void positionGlobalCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        position_global_pub->publish(*msg);
    }

    void originGlobalCallback(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg) {
        origin_global_pub->publish(*msg);
    }

    void velocityLocalCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        velocity_local_pub->publish(*msg);
    }

    void rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg) {
        rc_pub->publish(*msg);
    }

    void headingCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        heading_pub->publish(*msg);
    }
   

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_local_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr position_local_pub;
    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr drone_state_pub;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr position_global_pub;
    rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_global_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_local_pub;
    rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_pub;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_local_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_local_sub;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr position_global_sub;
    rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_global_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_local_sub;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FCInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
