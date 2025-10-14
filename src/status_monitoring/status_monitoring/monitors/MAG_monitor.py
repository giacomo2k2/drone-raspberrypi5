from rclpy.node import Node
from sensor_msgs.msg import MagneticField



class MAGMonitor:
    def __init__(self, node: Node, timeout_threshold=1.0):
        self.node = node
        self.timeout_threshold = timeout_threshold
        
        self.magnetic_field = MagneticField()


        self.nominal = None
        self.covariance_unknown = None
        self.last_update = None
        self.stale = None
        

        self.mag_sub = self.node.create_subscription(MagneticField, '/drone/fc_interface/mag', self.magcb, 10)



    def magcb(self, msg: MagneticField):
        self.stale = self.is_stale()
        self.last_update = self.node.get_clock().now()
        self.magnetic_field = msg


    def is_stale(self):
        if self.last_update is None:
            return True
        age = (self.node.get_clock().now().nanoseconds - self.last_update.nanoseconds) / 1e9
        return age > self.timeout_threshold
    



    def update(self):
        if all(cov == 0 for cov in self.magnetic_field.magnetic_field_covariance):
            self.covariance_unknown = True
        else:
            self.covariance_unknown = False


        self.nominal = not(self.covariance_unknown or self.stale)
        status = "NOMINAL" if self.nominal else "CRITICAL" 

        return {
            "status": status,
            "MAG_covariance_unkown": self.covariance_unknown,
            "MAG_stale": self.stale
        }