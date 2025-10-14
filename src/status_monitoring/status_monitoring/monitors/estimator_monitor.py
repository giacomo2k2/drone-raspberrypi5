from rclpy.node import Node
from mavros_msgs.msg import EstimatorStatus


class EstimatorMonitor:
    def __init__(self, node: Node, timeout_threshold = 1.0):
        self.node = node

        self.timeout_threshold = timeout_threshold

        self.estimator_status = EstimatorStatus() 


        self.nominal = None
        self.last_update = None
        self.stale = None

        self.estimator_sub = self.node.create_subscription(EstimatorStatus, '/drone/fc_interface/estimator', self.estimatorcb, 10)


    def estimatorcb(self, msg: EstimatorStatus):
        self.stale = self.is_stale()
        self.last_update = self.node.get_clock().now()
        
        self.estimator_status = msg


    def is_stale(self):
        if self.last_update is None:
            return True
        age = (self.node.get_clock().now().nanoseconds - self.last_update.nanoseconds) / 1e9
        return age > self.timeout_threshold
    


    def update(self): 
        self.nominal = not(self.estimator_status.attitude_status_flag or
                           self.estimator_status.velocity_horiz_status_flag or
                           self.estimator_status.velocity_vert_status_flag or
                           self.estimator_status.pos_horiz_rel_status_flag or
                           self.estimator_status.pos_horiz_abs_status_flag or
                           self.estimator_status.pos_vert_abs_status_flag or
                           self.estimator_status.pos_vert_agl_status_flag or
                           not(self.estimator_status.const_pos_mode_status_flag) or
                           not(self.estimator_status.gps_glitch_status_flag) or
                           not(self.estimator_status.accel_error_status_flag))
        
        status = "NOMINAL" if self.nominal else "CRITICAL" 

        return {
            "status": status, 
            "no_attitude": not(self.estimator_status.attitude_status_flag),
            "no_horiz_velocity": not(self.estimator_status.velocity_horiz_status_flag), 
            "no_vert_velocity": not(self.estimator_status.velocity_vert_status_flag), 
            "no_horiz_rel_pos": not(self.estimator_status.pos_horiz_rel_status_flag), 
            "no_horiz_abs_pos": not(self.estimator_status.pos_horiz_abs_status_flag), 
            "no_vert_abs_pos": not(self.estimator_status.pos_vert_abs_status_flag), 
            "estimator_frozen": self.estimator_status.const_pos_mode_status_flag, 
            "gps_glitch": self.estimator_status.gps_glitch_status_flag, 
            "imu_glitch": self.estimator_status.accel_error_status_flag,
        } 