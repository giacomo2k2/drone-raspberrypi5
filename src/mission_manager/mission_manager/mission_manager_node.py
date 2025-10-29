import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point


from rclpy.action import ActionClient


from drone_interfaces.srv import Disarm
from drone_interfaces.action import FollowTrajectory

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Subscribe to drone flight mode
        self.mode_sub = self.create_subscription(
            String, 
            '/drone/state_machine/flight_mode', 
            self.mode_cb, 
            10
        )
        self.current_mode = "IDLE"

        # Service client
        self.disarm_client = self.create_client(Disarm, '/drone/state_machine/disarm_srv')

        # Action client
        self.navigator_action_client = ActionClient(self, FollowTrajectory, 'drone/navigator/follow_trajectory')

        # Dummy mission: waypoints, hold times, and hold index
        self.waypoints = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=0.0, y=0.0, z=10.0),
            Point(x=0.0, y=15.0, z=10.0),
            Point(x=0.0, y=15.0, z=0.0),
        ]
        self.hold_times = [0, 2, 2, 0]
        self.hold_indexes = [0, 1, 2, 3]

        self.use_camera = True

        # State
        self.mission_started = False

        # Main loop timer
        self.timer = self.create_timer(1.0, self.mission_loop)

    def mode_cb(self, msg):
        self.current_mode = msg.data

    def mission_loop(self):
        # Only start mission when drone is in FLY mode and mission not started
        if self.current_mode == "ARMED" and not self.mission_started:
            self.get_logger().info("Starting mission...")
            self.mission_started = True
            self.send_navigation_goal()

    def send_navigation_goal(self):
        # Wait until Navigator action server is available
        if not self.navigator_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigator action server not available")
            return

        # Create goal
        goal_msg = FollowTrajectory.Goal()
        # Fill goal with waypoints, hold times, and hold index
        # Example: you can use custom fields in your FollowTrajectory action
        goal_msg.waypoints = self.waypoints
        goal_msg.hold_times = self.hold_times
        goal_msg.hold_indexes = self.hold_indexes
        goal_msg.use_camera = self.use_camera

        self.get_logger().info("Sending navigation goal to Navigator...")

        # Send goal
        self._send_goal_future = self.navigator_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigator_feedback_cb
        )
        self._send_goal_future.add_done_callback(self.navigator_goal_response)

    def navigator_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigator goal rejected")
            return

        self.get_logger().info("Navigator goal accepted, executing...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigator_result_cb)

    def navigator_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Example feedback: current position, segment, progress, etc.
        self.get_logger().info(f"nav feedback: "
                               f"type: {feedback.segment_type}, "
                               f"Pos: x={feedback.current_point.x:.2f}, y={feedback.current_point.y:.2f}, z={feedback.current_point.z:.2f} | "
                               f"Set: x={feedback.setpoint.x:.2f}, y={feedback.setpoint.y:.2f}, z={feedback.setpoint.z:.2f} | "
                               f"Seg: minor={feedback.current_minor_segment}, major={feedback.current_major_segment} | "
                               f"Prog: local={feedback.local_progress:.2f}, global={feedback.global_progress:.2f}, deviation={feedback.deviation:.2f}"
                                )

    def navigator_result_cb(self, future):
        result = future.result().result
        self.get_logger().info("Navigator action finished!")

        if result.success:
            self.get_logger().info("Mission completed successfully!")

            # Disarm the drone
            if not self.disarm_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Disarm service not available")
                return
            req = Disarm.Request()
            future = self.disarm_client.call_async(req)

            # Blocking wait for disarm to complete
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res.success:
                self.get_logger().info("Disarm request succeeded")
            else:
                self.get_logger().error("Disarm request failed")

        else:
            self.get_logger().error("Mission failed!")


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
