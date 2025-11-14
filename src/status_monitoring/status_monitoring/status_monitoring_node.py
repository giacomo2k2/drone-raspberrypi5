import rclpy
from rclpy.node import Node


# import monitors
from status_monitoring.monitors.PWR_monitor import PWRMonitor
from status_monitoring.monitors.GPS_monitor import GPSMonitor
from status_monitoring.monitors.IMU_monitor import IMUMonitor
from status_monitoring.monitors.MAG_monitor import MAGMonitor
from status_monitoring.monitors.estimator_monitor import EstimatorMonitor

from drone_interfaces.msg import FailsafeEvent, PreArmCheck, ComponentsStatus, SingleComponentStatus



# /mavros/altitude
# /mavros/estimator_status
# /mavros/extended_state
# /mavros/home_position/home
# /mavros/mag_calibration/status



class StatusMonitoringNode (Node):
    def __init__(self):
        super().__init__('status_monitoring')


        self.statuses = None

        self.monitors = {
            'PWR': PWRMonitor(self),
            'GPS': GPSMonitor(self),
            'IMU': IMUMonitor(self),
            'MAG': MAGMonitor(self, timeout_threshold=5.0),
            'ESTIMATOR': EstimatorMonitor(self),
        }

        self.components_status = ComponentsStatus()
        for monitor in self.monitors.keys():
            single_component_status = SingleComponentStatus()
            single_component_status.component_name = monitor
            single_component_status.status = "NOMINAL"
            self.components_status.components_status.append(single_component_status)


        # publishers
        #self.failsafe_pub = self.create_publisher(FailsafeEvent, '/drone/status_monitoring/failsafe', 10)
        self.prearm_pub = self.create_publisher(PreArmCheck, '/drone/status_monitoring/prearm', 10)
        self.components_status_pub = self.create_publisher(ComponentsStatus, '/drone/status_monitoring/components_status', 10)

    
        # timer
        self.timer = self.create_timer(1, self.monitor_loop_cb)
        

   

    def update(self):
        self.statuses = {key: monitor.update() for key, monitor in self.monitors.items()}
         

    def monitor_loop_cb(self):
        self.update()

        if self.statuses is None: 
            return
        
        prearm_msg = PreArmCheck()

    
        checks_passed = True

        prearm_msg.failed_components = []
        for key, result in self.statuses.items():

            for component_status in self.components_status.components_status:
                if component_status.component_name == key:
                    component_status.status = result['status']
           
            if self.statuses[key]['status'] == "CRITICAL":
                checks_passed = False
                self.pre_arm_good = False
                for check, value in result.items():
                    if check != "status" and value:
                        prearm_msg.failed_components.append(check)

        prearm_msg.checks_passed = checks_passed
        now = self.get_clock().now()
        prearm_msg.timestamp = now.to_msg()
        self.components_status.timestamp = now.to_msg()
        prearm_msg.header.stamp = now.to_msg()
        self.components_status.header.stamp = now.to_msg()

        self.prearm_pub.publish(prearm_msg)
        self.components_status_pub.publish(self.components_status)


def main(args = None):
    rclpy.init(args = args)
    node = StatusMonitoringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    