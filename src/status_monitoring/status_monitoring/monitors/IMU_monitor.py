from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUMonitor:
    def __init__(self, node: Node, timeout_threshold=1.0):
        self.node = node

        self.timeout_threshold = timeout_threshold

        # Imu
        self.imu = Imu()

        self.nominal = None
        self.covariance_unknown = None
        self.orientation_unknown = None
        self.angular_velocity_unknown = None
        self.linear_acceleration_unknown = None
        self.last_update = None
        self.stale = None

        self.imu_sub = self.node.create_subscription(Imu, '/drone/fc_interface/imu', self.imucb, 10)
        

    def imucb(self, msg:Imu):
        self.stale = self.is_stale()
        self.last_update = self.node.get_clock().now()
       
        self.imu = msg
        


    def is_stale(self):
        if self.last_update is None:
            return True
        age = (self.node.get_clock().now().nanoseconds - self.last_update.nanoseconds) / 1e9
        return age > self.timeout_threshold


    def update(self):

        if all(cov == 0 for cov in self.imu.orientation_covariance) or all(cov == 0 for cov in self.imu.angular_velocity_covariance) or all(cov == 0 for cov in self.imu.linear_acceleration_covariance):
            self.covariance_unknown = True
        else:
            self.covariance_unknown = False 

        if self.imu.orientation_covariance[0] == -1:
            self.orientation_unknown = True
        else:
            self.orientation_unknown = False

        if self.imu.angular_velocity_covariance[0] == -1:
            self.angular_velocity_unknown = True
        else:
            self.angular_velocity_unknown = False

        if self.imu.linear_acceleration_covariance[0] == -1:
            self.linear_acceleration_unknown = True
        else:
            self.linear_acceleration_unknown = False
    

        self.nominal = not(self.covariance_unknown or self.orientation_unknown or self.angular_velocity_unknown or self.linear_acceleration_unknown or self.stale)
        status = "NOMINAL" if self.nominal else "CRITICAL" 
        return {
            "status": status,
            "IMU_covariance_unknown": self.covariance_unknown,
            "IMU_orientation_unknown": self.orientation_unknown,
            "IMU_angular_velocity_unknown": self.angular_velocity_unknown,
            "IMU_linear_acceleration_unknown": self.linear_acceleration_unknown,
            "IMU_stale": self.stale
        }