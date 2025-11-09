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

from drone_interfaces.msg import ApriltagPosition


CAMERA = 'camera'

class ApriltagGuidanceNode(Node):

    def __init__(self):
        super().__init__('apriltag_guidance_node') 
        
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.apriltag_distance_pub = self.create_publisher(ApriltagPosition, '/drone/apriltag_guidance/apriltag_distance', 10)

        self.sub_tags = self.create_subscription(AprilTagDetectionArray, f'/{CAMERA}/detections', self.tags_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, f'/{CAMERA}/camera_info', self.info_callback, qos_profile)

        self.camera_info = None
        self.detections = []

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
        msg.center.x = self.center_cam[0]
        msg.center.y = self.center_cam[1]
        msg.center.z = self.center_cam[2]
        msg.timeout_expired = self.timeout_expired

        self.apriltag_distance_pub.publish(msg)


    def tags_callback(self, msg):

        if CameraInfo == None:
            return
         
        self.detections = msg.detections  
        self.start_time = self.get_clock().now()

        for det in self.detections:
            # draw 6D pose axes
            try: 
                t = self.tf_buffer.lookup_transform(CAMERA, f'tag36h11:{det.id}', rclpy.time.Time())

            except (tf2.LookupException, tf2.ExtrapolationException):
                continue

            # transform points for axes
            q = t.transform.rotation
            r = R.from_quat([q.x, q.y, q.z, q.w])
            R_mat = r.as_matrix()
            t_vec = np.array([t.transform.translation.x,
                              t.transform.translation.y,
                              t.transform.translation.z])
            
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
