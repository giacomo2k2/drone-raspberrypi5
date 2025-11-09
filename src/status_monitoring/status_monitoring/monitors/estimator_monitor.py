from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

class EstimatorMonitor:
    def __init__(self, node: Node, timeout_threshold = 1.0):
        self.node = node

        self.timeout_threshold = timeout_threshold

        self.local_position = None 
        self.local_velocity = None

        self.nominal = None
        self.last_update_pos = None
        self.last_update_vel = None
        self.stale_pos = None
        self.stale_vel = None

        self.local_position_sub = self.node.create_subscription(PoseStamped, '/drone/fc_interface/position_local', self.local_position_cb, 10)
        self.local_velocity_sub = self.node.create_subscription(TwistStamped, '/drone/fc_interface/velocity_local', self.local_velocity_cb, 10)

    def local_position_cb(self, msg: PoseStamped):
        self.stale_pos = self.is_stale_pos()
        self.last_update_pos = self.node.get_clock().now()

        self.local_position = msg

    def local_velocity_cb(self, msg: TwistStamped):
        self.stale_vel = self.is_stale_vel()
        self.last_update_vel = self.node.get_clock().now()

        self.local_velocity = msg

    def is_stale_pos(self):
        if self.last_update_pos is None:
            return True
        age = (self.node.get_clock().now().nanoseconds - self.last_update_pos.nanoseconds) / 1e9
        return age > self.timeout_threshold
    
    def is_stale_vel(self):
        if self.last_update_vel is None:
            return True
        age = (self.node.get_clock().now().nanoseconds - self.last_update_vel.nanoseconds) / 1e9
        return age > self.timeout_threshold

    def update(self):
        self.nominal = not self.stale_pos and not self.stale_vel and self.local_position and self.local_velocity
        
        status = "NOMINAL" if self.nominal else "CRITICAL" 

        return {
            "status": status, 
            "no_position": self.local_position is None or self.stale_pos,
            "no_velocity": self.local_velocity is None or self.stale_vel,
        } 