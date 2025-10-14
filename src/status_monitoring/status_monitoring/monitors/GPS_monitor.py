# GPS accuracy / fix quality



from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSMonitor:
    def __init__(self, node: Node, timeout_threshold=1.0):
        self.node = node

        self.timeout_threshold = timeout_threshold

        # GPS fix
        self.nav_sat_fix = NavSatFix()


        self.nominal = None
        self.no_fix = None
        self.no_service = None
        self.accuracy_unknown = None
        self.last_update = None
        self.stale = None

        self.fix_sub = self.node.create_subscription(NavSatFix, '/drone/fc_interface/fix', self.fixcb, 10)
    

    def fixcb(self, msg: NavSatFix):
        self.stale = self.is_stale()
        self.last_update = self.node.get_clock().now()

        self.nav_sat_fix = msg



    def is_stale(self):
        if self.last_update is None:
            return True
        age = (self.node.get_clock().now().nanoseconds - self.last_update.nanoseconds) / 1e9
        return age > self.timeout_threshold


    def update(self):
        
    
        if self.nav_sat_fix.status.status == -1:
            self.no_fix = True
        else:
            self.no_fix = False

        if self.nav_sat_fix.status.service & 1 == 0:
            self.no_service = True
        else:
            self.no_service = False

        if self.nav_sat_fix.position_covariance_type == 0:
            self.accuracy_unknown = True
        else:
            self.accuracy_unknown = False



        self.nominal = not(self.no_fix or self.no_service or self.accuracy_unknown or self.stale)
        status = "NOMINAL" if self.nominal else "CRITICAL" 
        return {
            "status": status,
            "GPS_no_fix": self.no_fix,
            "GPS_no_service": self.no_service,
            "GPS_accuracy_unknown": self.accuracy_unknown,
            "GPS_stale": self.stale
        }