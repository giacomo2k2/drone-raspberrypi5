import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from drone_interfaces.msg import PreArmCheck

from enum import Enum

class FM(Enum):
    IDLE = 0
    CHANGE_MODE = 1
    PREARM_CHECK = 2
    ARM = 3
    FLY = 4

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')

        # Publishers
        self.mode_pub = self.create_publisher(String, 'drone/state_machine/flight_mode', 10)

        # Subscribers
        self.state_sub = self.create_subscription(State, '/drone/fc_interface/state', self.state_cb, 10)
        self.prearm_sub = self.create_subscription(PreArmCheck, '/drone/status_monitoring/prearm', self.prearm_cb, 10)

        #  clients
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')

        # State machine vars
        self.fc_state = None
        self.prearm = None
        self.mode = FM.IDLE
        self._mode_requested = False
        self._arm_requested = False

        # Main loop timer
        self.timer = self.create_timer(1.0, self.state_loop)

    def prearm_cb(self, msg):
        self.prearm = msg

    def state_cb(self, msg):
        self.fc_state = msg

    def state_loop(self):
        if not self.fc_state or not self.fc_state.connected:
            self.get_logger().info("Waiting for MAVROS connection...")
            return

        # ---------- IDLE ----------
        if self.mode == FM.IDLE:
            self.get_logger().info("MAVROS connection established")

            if self.fc_state.mode == "Guided":
                self.get_logger().info("Mode is already GUIDED")
                self.mode = FM.PREARM_CHECK
            else:
                self.mode = FM.CHANGE_MODE

        # ---------- CHANGE_MODE ----------
        elif self.mode == FM.CHANGE_MODE:
            if not self._mode_requested:
                self.change_mode("GUIDED")
                self._mode_requested = True
            elif self.fc_state.mode == 'GUIDED':
                self.get_logger().info("Mode changed to GUIDED")
                self.mode = FM.PREARM_CHECK
                self._mode_requested = False


        # ---------- PREARM_CHECK ---------
        elif self.mode == FM.PREARM_CHECK:
            self.get_logger().info("Checking prearm conditions...")
            if self.prearm and self.prearm.checks_passed:
                self.get_logger().info("prearm checks passed, arming...")
                self.mode = FM.ARM
            else:
                if self.prearm: 
                    self.get_logger().warn(f"waiting for pre arm check to pass. : {', '.join(self.prearm.failed_components)}")
                else:
                    self.get_logger().warn("Waiting for prearm message...")

        # ---------- ARM ----------
        elif self.mode == FM.ARM:
            if self.fc_state.armed:
                self.get_logger().info("armed, moving to TAKEOFF")
                self.mode = FM.TAKEOFF
                self._arm_requested = False

            else:
                if not self._arm_requested:
                    self.get_logger().info("sending arming request")
                    self.arm(True)       
                    self._arm_requested = True   
                else:
                    self.get_logger().info("waiting for FC to arm...")
    


        # ---------- TAKEOFF ----------
        elif self.mode == FM.FLY:
            self.get_logger().info("Flying")
            



        # Publish current mode for other nodes
        msg = String()
        msg.data = self.mode.name
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Flight mode: {self.mode.name}")


    def arm(self, value: bool):
        if not self.arming_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arming service not available")
            return
        req = CommandBool.Request()
        req.value = value
        future = self.arming_cli.call_async(req)

        # Non-blocking wait
        def callback(fut):
            res = fut.result()
            if res.success:
                self.get_logger().info("Arming request succeeded")
            else:
                self.get_logger().error("Arming request failed")

        future.add_done_callback(callback)

    def change_mode(self, mode: str):
        if not self.mode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("SetMode service not available")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_cli.call_async(req)

        # Non-blocking wait
        def callback(fut):
            res = fut.result()
            if res.mode_sent:
                self.get_logger().info(f"Mode change request to {mode} sent successfully")
            else:
                self.get_logger().error(f"Mode change request to {mode} failed")

        future.add_done_callback(callback)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
