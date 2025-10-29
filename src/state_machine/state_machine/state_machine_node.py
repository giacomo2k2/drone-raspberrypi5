from unittest import case
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from drone_interfaces.msg import PreArmCheck

from drone_interfaces.srv import Disarm, Land, Takeoff

from mavros_msgs.srv import CommandTOL

from enum import Enum



class FlightProcedures(Enum):
    IDLE = 0x00
    WAIT_MAVROS_CONNECTED = 0x01
    PREARM_CHECK = 0x02
    CHANGE_MODE_GUIDED = 0x03
    ARM = 0x04
    TAKEOFF = 0x05
    CHANGE_MODE_LAND = 0x06
    DISARM = 0x07

class FlightMode(Enum):
    IDLE = 0x00
    ARMING = 0x10
    ARMED = 0x20
    DISARMED = 0x30


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
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # server
        self.disarm_srv = self.create_service(Disarm, '/drone/state_machine/disarm_srv', self.disarm_cb)
        self.takeoff_srv = self.create_service(Takeoff, '/drone/state_machine/takeoff_srv', self.takeoff_cb)
        self.land_srv = self.create_service(Land, '/drone/state_machine/land_srv', self.land_cb)

        # State machine vars
        self.fc_state = None
        self.prearm = None

        self.procedure = FlightProcedures.IDLE
        self.mode = FlightMode.IDLE

        self._mode_requested = False
        self._arm_requested = False
    
        # takeoff
        self._takeoff_requested = False
        self._takeoff_altitude = None

        # Main loop timer
        self.timer = self.create_timer(1.0, self.state_loop)

    def prearm_cb(self, msg):
        self.prearm = msg

    def state_cb(self, msg):
        self.fc_state = msg

    def disarm_cb(self, request, response):
        self.get_logger().info(f"Switching to DISARM procedure")
        self.procedure = FlightProcedures.DISARM
        response.success = True
        return response
    
    def land_cb(self, request, response):
        self.get_logger().info(f"Switching to LAND procedure")
        self.procedure = FlightProcedures.CHANGE_MODE_LAND
        response.success = True
        return response
    
    def takeoff_cb(self, request, response):
        self.get_logger().info(f"Switching to TAKEOFF, altutude requested: {request.altitude}")

        self.takeoff_altitude = request.altitude
        self.procedure = FlightProcedures.TAKEOFF
        response.success = True
        return response

    def state_loop(self):
        
        ########### MODE STATE MACHINE ###########

        match self.mode:
            case FlightMode.IDLE:
                self.procedure = FlightProcedures.WAIT_MAVROS_CONNECTED
                self.mode = FlightMode.ARMING
            case FlightMode.ARMING:
                pass
            case FlightMode.ARMED:
                pass

            case FlightMode.DISARMED:
                pass

        ########### PROCEDURE STATE MACHINE ###########

        match self.procedure:
            case FlightProcedures.IDLE:
                self.get_logger().info("Waiting for commands...")

            case FlightProcedures.WAIT_MAVROS_CONNECTED:
                if self.fc_state and self.fc_state.connected:
                    self.get_logger().info("MAVROS connection established.")
                    self.procedure = FlightProcedures.PREARM_CHECK
                else:
                    self.get_logger().info("Waiting for MAVROS connection...")
                
            case FlightProcedures.PREARM_CHECK:
                if self.prearm and self.prearm.checks_passed:
                    self.get_logger().info("Prearm checks passed. ")
                    self.procedure = FlightProcedures.CHANGE_MODE_GUIDED    
                else:
                    if self.prearm:
                        self.get_logger().warn(f"Waiting for prearm check to pass: {', '.join(self.prearm.failed_components)}")
                    else:
                        self.get_logger().warn("Waiting for prearm message...")

            case FlightProcedures.CHANGE_MODE_GUIDED:
                if self.fc_state.mode.upper() == 'GUIDED':
                    self.get_logger().info("Mode changed to GUIDED, arming...")
                    self.procedure = FlightProcedures.ARM
                    self._mode_requested = False
                else:
                    if not self._mode_requested:
                        self.get_logger().info("Requesting Guided mode...")
                        self._mode_requested = True
                        self.change_mode("GUIDED")
                    else:
                        self.get_logger().info("Waiting for mode change to GUIDED...") 

            case FlightProcedures.ARM:
                if self.fc_state.armed:
                    self.get_logger().info("armed, moving to FLY mode")
                    self.mode = FlightMode.ARMED
                    self.procedure = FlightProcedures.IDLE
                    self._arm_requested = False
                else:
                    if not self._arm_requested:
                        self.get_logger().info("sending arming request")
                        self.arm(True)       
                        self._arm_requested = True   
                    else:
                        self.get_logger().info("waiting for FC to arm...")

            case FlightProcedures.TAKEOFF:
                    if not self._takeoff_requested:
                        self.get_logger().info(f"Requesting takeoff to {self.takeoff_altitude} meters...")
                        self.takeoff(self.takeoff_altitude)
                        self._takeoff_requested = True
                    else:
                        self._takeoff_requested = False
                        self.procedure = FlightProcedures.IDLE

            case FlightProcedures.CHANGE_MODE_LAND: 
                if not self._mode_requested:
                    self.get_logger().info("Requesting Landing mode...")
                    self._mode_requested = True
                    self.change_mode("LAND")
                else:
                    if self.fc_state.mode.upper() != 'LAND':
                        self.get_logger().info("Waiting for mode change to LAND...")
                    elif self.fc_state.mode.upper() == 'LAND':
                        self.get_logger().info("Mode changed to LAND")
                        self._mode_requested = False
                        self.procedure = FlightProcedures.IDLE
                                
            case FlightProcedures.DISARM:
                if not self.fc_state.armed:
                    self.get_logger().info("Disarmed successfully")
                    self.mode = FlightMode.DISARMED
                    self.procedure = FlightProcedures.IDLE
                    self._arm_requested = False
                else:            
                    if not self._arm_requested:
                        self.get_logger().info("sending disarming request")
                        self.arm(False)       
                        self._arm_requested = True   
                    else:
                        self.get_logger().info("waiting for FC to disarm...")
            


        # Publish current mode for other nodes
        msg = String()
        msg.data = self.mode.name
        self.mode_pub.publish(msg)
        self.get_logger().info(f"Mode: {self.mode.name}, Procedure: {self.procedure.name}")


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

    def takeoff(self, altitude: float):
        if not self.takeoff_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Takeoff service not available")
            return
        
        req = CommandTOL.Request()
        req.altitude = altitude

        future = self.takeoff_cli.call_async(req)

        # Non-blocking wait
        def callback(fut):
            res = fut.result()
            if res.success:
                self.get_logger().info("Takeoff request sent successfully")
            else:
                self.get_logger().error("Takeoff request failed")

        future.add_done_callback(callback)

     


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
