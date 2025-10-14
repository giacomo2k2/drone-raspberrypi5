import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from example_interfaces.action import FollowTrajectory  # Replace with your actual action type

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

        # Action client for Navigator
        self.navigator_action_client = ActionClient(self, FollowTrajectory, 'follow_trajectory')

        # Dummy mission: waypoints, hold times, and hold index
        self.waypoints = [
            Point(x=0.0, y=0.0, z=1.0),
            Point(x=5.0, y=0.0, z=1.0),
            Point(x=5.0, y=5.0, z=1.0),
            Point(x=0.0, y=5.0, z=1.0),
            Point(x=0.0, y=0.0, z=1.0)
        ]
        self.hold_times = [0, 2, 0, 0, 0]  # seconds to hold at each waypoint
        self.hold_index = 1  # index where the drone should pause

        # State
        self.mission_started = False

        # Main loop timer
        self.timer = self.create_timer(1.0, self.mission_loop)

    def mode_cb(self, msg):
        self.current_mode = msg.data

    def mission_loop(self):
        # Only start mission when drone is in FLY mode and mission not started
        if self.current_mode == "FLY" and not self.mission_started:
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
        goal_msg.hold_index = self.hold_index

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
        self.get_logger().info(f"Navigator feedback: x={feedback.current_point.x:.2f}, "
                               f"y={feedback.current_point.y:.2f}, z={feedback.current_point.z:.2f}")

    def navigator_result_cb(self, future):
        result = future.result().result
        self.get_logger().info("Navigator action finished!")
        # Optionally, you can start next mission here

def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
