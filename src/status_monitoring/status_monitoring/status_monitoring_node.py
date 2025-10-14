import rclpy
from rclpy.node import Node
from mavros_msgs.msg import StatusText

# import monitors
from status_monitoring.monitors.PWR_monitor import PWRMonitor
from status_monitoring.monitors.GPS_monitor import GPSMonitor
from status_monitoring.monitors.IMU_monitor import IMUMonitor
from status_monitoring.monitors.MAG_monitor import MAGMonitor
from status_monitoring.monitors.estimator_monitor import EstimatorMonitor

from drone_interfaces.msg import FailsafeEvent, PreArmCheck, Status




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
        }

        self.pre_arm_good = False

        # publishers
        #self.failsafe_pub = self.create_publisher(FailsafeEvent, '/drone/status_monitoring/failsafe', 10)
        self.prearm_pub = self.create_publisher(PreArmCheck, '/drone/status_monitoring/prearm', 10)
        #self.status_pub = self.create_publisher(Status, '/drone/status_monitoring/status', 10)

        # subscribers
        self.status_text_sub = self.create_subscription(StatusText, '/drone/fc_interface/status_text', self.status_text_cb, 10)

        # timer
        self.timer = self.create_timer(1, self.monitor_loop_cb)
        

    def status_text_cb(self, msg: StatusText):
        text = msg.text

        if "EKF3 IMU1 is using GPS" == text:
            self.pre_arm_good = True


    def update(self):
        self.statuses = {key: monitor.update() for key, monitor in self.monitors.items()}


    def monitor_loop_cb(self):
        self.update()

        if self.statuses is None: 
            return
        
        prearm_msg = PreArmCheck()

        #################### PAY ATTENTION to pre arm good (for SITL only )############################
        checks_passed = self.pre_arm_good

        prearm_msg.failed_components = []

        if not self.pre_arm_good:
            prearm_msg.failed_components.append("waiting for SITL pre arm good")

        for key, result in self.statuses.items():
            if self.statuses[key]['status'] == "CRITICAL":
                checks_passed = False
                self.pre_arm_good = False
                for check, value in result.items():
                    if check != "status" and value:
                        prearm_msg.failed_components.append(check)

        prearm_msg.checks_passed = checks_passed

        prearm_msg.timestamp = self.get_clock().now().to_msg()
        
        self.prearm_pub.publish(prearm_msg)



def main(args = None):
    rclpy.init(args = args)
    node = StatusMonitoringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    