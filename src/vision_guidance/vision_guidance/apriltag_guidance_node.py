import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import tf2_ros
import tf2_py as tf2
from scipy.spatial.transform import Rotation as R
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64
from drone_interfaces.msg import ApriltagPosition


from rclpy.time import Time
from rclpy.duration import Duration



class ApriltagGuidanceNode(Node):

    def __init__(self):
        super().__init__('apriltag_guidance_node')

        self.declare_parameter('camera_frame', 'camera') 
        self.camera_frame = self.get_parameter('camera_frame').value

        self.declare_parameter('camera_name', 'camera')
        self.camera_name = self.get_parameter('camera_name').value

        self.declare_parameter('platform', "CC")

        self.platform = self.get_parameter('platform').value

        if self.platform == "CC":
            self.z_rot = np.pi
        else:
            self.z_rot = np.pi

        
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.apriltag_distance_pub = self.create_publisher(ApriltagPosition, '/drone/apriltag_guidance/apriltag_distance', 10)

        self.sub_tags = self.create_subscription(AprilTagDetectionArray, f'/{self.camera_name}/detections', self.tags_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, f'/{self.camera_name}/camera_info', self.info_callback, qos_profile)
        self.heading_sub = self.create_subscription(Float64, "/drone/fc_interface/heading", self.heading_callback, 10)

        self.camera_info = None
        self.detections = []
        self.heading = None
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timeout = 1
        self.start_time = self.get_clock().now()

        self.timeout_expired = False

        self.center_cam = None

        self.timer = self.create_timer(0.05, self.main_loop)
        

    def main_loop(self): 

        if self.center_cam is None: 
            return
        
        self.timeout_expired = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 > self.timeout
        
        msg = ApriltagPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.center.x = self.center_cam[0]
        msg.center.y = self.center_cam[1]
        msg.center.z = self.center_cam[2]
        msg.timeout_expired = self.timeout_expired

    
        self.apriltag_distance_pub.publish(msg)

    def heading_callback(self, msg):
        self.heading = msg.data

    def tags_callback(self, msg):

        self.detections = msg.detections  

        if self.camera_info is None or len(self.detections) == 0:
            return
         
        
        self.start_time = self.get_clock().now()

        for det in self.detections:
            # draw 6D pose axes
            try:
                # Lookup latest transform, waiting up to 0.5 seconds
                t = self.tf_buffer.lookup_transform(
                    self.camera_frame, 
                    f'tag36h11:{det.id}', 
                    Time(),  # latest available
                    timeout=Duration(seconds=0.5)
                )
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
                self.get_logger().warn(f"TF lookup failed for tag {det.id}: {e}")
                continue



            # transform points for axes
            q = t.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            R_mat = r.as_matrix()
            t_vec = np.array([t.transform.translation.x,
                              t.transform.translation.y,
                              t.transform.translation.z])
            
            R_z_rot = R.from_euler('z', self.z_rot).as_matrix()
            R_mat = R_z_rot @ R_mat
            t_vec = R_z_rot @ t_vec

            if self.heading is not None:
                heading_rad = np.deg2rad(self.heading)
                R_heading = R.from_euler('z', heading_rad).as_matrix()
                R_mat = R_heading @ R_mat
                t_vec = R_heading @ t_vec
            
            center_3d = np.array([0, 0, 0])

            self.center_cam = (R_mat @ center_3d.T) + t_vec   # center wrt camera ref frame

            
            
            
            
    def info_callback(self, msg): 
        self.camera_info = msg
        
    
    

def main(args=None):
    rclpy.init(args=args)
    node = ApriltagGuidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
