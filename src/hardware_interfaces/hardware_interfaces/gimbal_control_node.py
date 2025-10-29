import rclpy
from rclpy.node import Node
from mavros_msgs.msg import GimbalManagerSetAttitude
from geometry_msgs.msg import Quaternion

class GimbalControlNode(Node):
    def __init__(self):
        super().__init__('fixed_gimbal_node')
        self.pub = self.create_publisher(GimbalManagerSetAttitude,
                                         '/mavros/gimbal_control/manager/set_attitude', 10)
        self.timer = self.create_timer(1.0, self.publish_downward)

        self.get_logger().info("Waiting to publish gimbal orientation...")

    def publish_downward(self):
        msg = GimbalManagerSetAttitude()
        msg.target_system = 1        # MAV system ID (default SITL: 1)
        msg.target_component = 1     # MAV component ID (gimbal usually: 1)
        msg.flags = 0                # Use 0 for full control
        msg.gimbal_device_id = 1     # Gimbal device ID in SITL: 1

        # Quaternion for downward-facing gimbal (pitch -90 deg)
        # Roll=0, Pitch=-pi/2, Yaw=0
        msg.q = Quaternion(x=0.0, y=-0.7071068, z=0.0, w=0.7071068)
        msg.angular_velocity_x = 0.0
        msg.angular_velocity_y = 0.0
        msg.angular_velocity_z = 0.0

        self.pub.publish(msg)
        self.get_logger().info("Gimbal commanded downward.")
        self.timer.cancel()  # publish once

def main(args=None):
    rclpy.init(args=args)
    node = GimbalControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
