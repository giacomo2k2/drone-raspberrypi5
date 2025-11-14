import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

from .geodetic_ENU import geodetic_to_enu

from rclpy.action import ActionClient

from drone_interfaces.msg import CompanionComputerSM, Mission
from drone_interfaces.srv import Arm, Disarm, StartMission, AbortMission
from drone_interfaces.action import FollowTrajectory

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Subscribe to drone flight mode
        self.SM_sub = self.create_subscription(CompanionComputerSM, '/drone/state_machine/state_machine', self.SM_cb, 10)
        self.origin_sub = self.create_subscription(GeoPointStamped, '/drone/fc_interface/global_origin', self.origin_cb, 10)
        
        # Publisher
        self.mission_pub = self.create_publisher(Mission, '/drone/mission_manager/mission', 10)

        #qos = rclpy.qos.QoSProfile(depth=10)
        #qos.reliable = True
        #qos.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL

        #self.safety_polygon_pub = self.create_publisher(PolygonStamped, '/drone/mission_manager/safety_polygon', qos)

        # Current flight mode
        self.current_mode = "IDLE"

        # Service client
        self.arm_client = self.create_client(Arm, '/drone/state_machine/arm_srv')
        self.disarm_client = self.create_client(Disarm, '/drone/state_machine/disarm_srv')

        # Action client
        self.navigator_action_client = ActionClient(self, FollowTrajectory, 'drone/navigator/follow_trajectory')

        # service server
        self.start_mission_server = self.create_service(
            StartMission,
            '/drone/mission_manager/start_mission_srv',
            self.start_mission_cb
        )

        self.abort_mission_server = self.create_service(
            AbortMission,
            '/drone/mission_manager/abort_mission_srv',
            self.abort_mission_cb
        )

        self.goal_handle = None

        self.mission_started = False

        self.origin = None
        self.latitudes = None
        self.longitudes = None
        self.altitudes = None
        self.relative_altitudes = None
        self.waypoints = []
        self.hold_times = None
        self.hold_indexes = None
        self.use_camera = None

        self.waiting_for_arming = False
        self.pending_mission_request = None

        self.mission_type = None
        self.mission_completed = False
        self.mission_progress = 0.0
        self.mission_start_time = None
        self.mission_eta = None

        # polygon
        self.safety_margin = 0.2 # 20% margin

        # timer 
        self.mission_timer = self.create_timer(1.0, self.mission_timer_cb)

    def SM_cb(self, msg):
        prev_mode = self.current_mode
        self.current_mode = msg.flight_mode

        # Detect transition to ARMED
        if self.waiting_for_arming and self.current_mode == "ARMED" and prev_mode != "ARMED":
            self.waiting_for_arming = False
            self.get_logger().info("Drone is now ARMED — resuming pending mission")

            # Restart the mission logic
            if self.pending_mission_request is not None:
                self._start_mission(self.pending_mission_request)
                self.pending_mission_request = None

    def origin_cb(self, msg):
        self.origin = msg

    def mission_timer_cb(self):
        if self.mission_started and self.mission_type:
            mission_msg = Mission()
            mission_msg.header.stamp = self.get_clock().now().to_msg()
            mission_msg.mission_type = self.mission_type
            mission_msg.mission_status = "in-progress"
            mission_msg.progress = self.mission_progress
            self.mission_eta = time.time() - self.mission_start_time
            mission_msg.eta = self.mission_eta
            self.mission_pub.publish(mission_msg)
        
        if self.mission_completed:
            mission_msg = Mission()
            mission_msg.header.stamp = self.get_clock().now().to_msg()
            mission_msg.mission_type = self.mission_type
            mission_msg.mission_status = "completed"
            mission_msg.progress = 1.0
            mission_msg.eta = self.mission_eta
            self.mission_pub.publish(mission_msg)
       

    def start_mission_cb(self, request, response):
        self.get_logger().info("Start mission request received")

        if self.mission_started:
            response.accepted = False
            response.message = "Mission already in progress"
            return response

        if self.current_mode == "PRE_ARMED":
            if not self.arm_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("Arm service not available")
                response.accepted = False
                response.message = "Arm service not available"
                return response

            self.get_logger().info("Drone not armed — sending arm request...")
            req = Arm.Request()
            req.trigger = True
            self.arm_client.call_async(req)

            # Store mission request to start automatically once armed
            self.waiting_for_arming = True
            self.pending_mission_request = request
            response.accepted = True
            response.message = "Arming in progress; mission will start once ARMED"
            return response

        elif self.current_mode == "ARMED":
            # If already armed → start mission immediately
            return self._start_mission(request)
        else: 
            response.accepted = False
            response.message = "Drone not pre-armed nor armed"
            return response
    
    def construct_polygon(self):
        max_x = max([wp.x for wp in self.waypoints])
        min_x = min([wp.x for wp in self.waypoints])
        max_y = max([wp.y for wp in self.waypoints])
        min_y = min([wp.y for wp in self.waypoints])
        max_z = max([wp.z for wp in self.waypoints])

        p1 = Point32()
        if min_x < 0:
                p1.x = min_x*(1+self.safety_margin)
        else:
            p1.x = min_x*(1-self.safety_margin)
        
        if min_y < 0:
            p1.y = min_y*(1+self.safety_margin)
        else:
            p1.y = min_y*(1-self.safety_margin)

            p1.z = -10.0  # Ground level

        p2 = Point32()
        if max_x < 0:
            p2.x = max_x*(1-self.safety_margin)
        else:
            p2.x = max_x*(1+self.safety_margin)

        if max_y < 0:
            p2.y = max_y*(1-self.safety_margin)
        else:
            p2.y = max_y*(1+self.safety_margin)

        if max_z < 0:
            p2.z = max_z*(1-self.safety_margin)
        else:
            p2.z = max_z*(1+self.safety_margin)

        return p1, p2

    def _start_mission(self, request):
        self.mission_started = True

        if request.mission_type == "setpoint-navigation":
            self.mission_type = "setpoint-navigation"

            setpoint_nav = request.mission_data.setpoint_navigation
            self.latitudes = setpoint_nav.latitudes
            self.longitudes = setpoint_nav.longitudes
            self.altitudes = setpoint_nav.altitudes
            self.relative_altitudes = setpoint_nav.relative_altitudes
            self.hold_times = setpoint_nav.hold_times
            self.hold_indexes = setpoint_nav.hold_indexes
            self.use_camera = setpoint_nav.precise_landing

            if not self.latitudes or not self.longitudes or not self.altitudes or not self.relative_altitudes or not self.origin:
                self.get_logger().error("Some mission data missing.")
                return False

            self.waypoints.clear()
            origin = self.origin.position
            for lat, lon, alt, rel_alt in zip(self.latitudes, self.longitudes, self.altitudes, self.relative_altitudes):
                wp = Point()
                wp.x, wp.y, _ = geodetic_to_enu(lat, lon, alt, origin.latitude, origin.longitude, origin.altitude)
                wp.z = rel_alt
                self.waypoints.append(wp)

            # Publish safety polygon
            #polygon_msg = PolygonStamped()
            #polygon_msg.header.stamp = self.get_clock().now().to_msg()
            #polygon_msg.header.frame_id = "local_origin"

            #p1, p2 = self.construct_polygon()
            #polygon_msg.polygon.points = [p1, p2]
            #self.safety_polygon_pub.publish(polygon_msg)
            
            self.get_logger().info("Mission waypoints computed, sending goal...")
            self.send_navigation_goal()
            return True

        else:
            self.get_logger().error(f"Unknown mission type: {request.mission_type}")
            return False
    

    def abort_mission_cb(self, request, response):
        self.get_logger().info("Abort mission request received")
        if self.mission_started:

            if self.goal_handle:
                cancel_future = self.goal_handle.cancel_goal_async()
                response.message = "Mission cancel request sent"
                response.success = True
                return response
            else:
                response.success = False
                response.message = "No active mission goal to cancel"
                return response

        else:
            response.success = False
            response.message = "No mission in progress to abort"
            return response

    def send_navigation_goal(self):
        # Wait until Navigator action server is available
        if not self.navigator_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigator action server not available")
            return

        # Create goal
        goal_msg = FollowTrajectory.Goal()
        
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

        self.mission_start_time = time.time()

        self._send_goal_future.add_done_callback(self.navigator_goal_response)

    def navigator_goal_response(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Navigator goal rejected")
            return

        self.get_logger().info("Navigator goal accepted, executing...")
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigator_result_cb)

    def navigator_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.mission_progress = feedback.global_progress

        # Example feedback: current position, segment, progress, etc.
        # self.get_logger().info(f"nav feedback: "
        #                        f"type: {feedback.segment_type}, "
        #                        f"Pos: x={feedback.current_point.x:.2f}, y={feedback.current_point.y:.2f}, z={feedback.current_point.z:.2f} | "
        #                        f"Set: x={feedback.setpoint.x:.2f}, y={feedback.setpoint.y:.2f}, z={feedback.setpoint.z:.2f} | "
        #                        f"Seg: minor={feedback.current_minor_segment}, major={feedback.current_major_segment} | "
        #                        f"Prog: local={feedback.local_progress:.2f}, global={feedback.global_progress:.2f}, deviation={feedback.deviation:.2f}"
        #                         )

    def navigator_result_cb(self, future):
        result = future.result().result
        
        self.get_logger().info("Navigator action finished!")

        if result.success:
            self.get_logger().info("Mission completed successfully!")   
        else:
            self.get_logger().error("Mission failed!")
    
        # Disarm the drone
        if not self.disarm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Disarm service not available")
            return
  
        self.disarm_client.call_async(Disarm.Request())

        self.get_logger().info("Disarm request succeeded")

def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
