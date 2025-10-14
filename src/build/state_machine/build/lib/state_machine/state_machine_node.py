import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')

        # Publishers
        self.mode_pub = self.create_publisher(String, '/flight_mode', 10)
        self.setpoint_pub = self.create_publisher(PoseStamped, '/final_setpoints', 10)

        # Subscribers
        self.state_sub = self.create_subscription(State, '/uav/state', self.state_cb, 10)

        # Service clients
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

        # State machine vars
        self.fc_state = None
        self.mode = 'IDLE'

        # Main loop timer
        self.timer = self.create_timer(2.0, self.state_loop)

    def state_cb(self, msg):
        self.fc_state = msg

    def state_loop(self):
        if self.mode == 'IDLE':
            # Request GUIDED mode
            self.change_mode("GUIDED")
            # Arm
            self.arm(True)
            self.mode = 'TAKEOFF'

        elif self.mode == 'TAKEOFF':
            # Publish a takeoff setpoint (z=2.0)
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.pose.position.z = 2.0
            self.setpoint_pub.publish(sp)
            self.mode = 'HOVER'

        elif self.mode == 'HOVER':
            # Stay here — in future you can add triggers to LAND etc.
            pass

        # Publish current mode for other nodes
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Flight mode: {self.mode}")

    def arm(self, value: bool):
        if not self.arming_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return
        req = CommandBool.Request()
        req.value = value
        future = self.arming_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Arming result: {future.result().success}")
        else:
            self.get_logger().error("Arming call failed")

    def change_mode(self, mode: str):
        if not self.mode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SetMode service not available")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Mode change to {mode}: {future.result().mode_sent}")
        else:
            self.get_logger().error("SetMode call failed")

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
