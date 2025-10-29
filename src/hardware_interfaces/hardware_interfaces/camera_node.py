import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import tf2_ros
import tf2_py as tf2
from scipy.spatial.transform import Rotation as R
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool

# The topic name is /gimbal_camera/image_raw, correctly bridged in the launch file.
CAMERA = 'gimbal_camera'

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node') # Renamed node to camera_node to match launch file
        
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        self.sub_image = self.create_subscription(Image, f'/{CAMERA}/image_raw', self.image_callback, 1)
        self.sub_tags = self.create_subscription(AprilTagDetectionArray, f'/{CAMERA}/detections', self.tags_callback, qos_profile)
        self.sub_info = self.create_subscription(CameraInfo, f'/{CAMERA}/camera_info', self.info_callback, qos_profile)
        self.detector_status_sub = self.create_subscription(Bool, '/drone/camera_calibration/detector_status', self.detector_status_cb, 10)

        self.camera_info = None
        self.latest_frame = None
        self.detections = []

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.detector_status = False
        
        cv2.namedWindow("Gimbal Camera Feed (SITL)", cv2.WINDOW_AUTOSIZE)

    def detector_status_cb(self, msg):
        self.detector_status = msg.data

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_frame()

    def tags_callback(self, msg):
        self.detections = msg.detections

    def info_callback(self, msg): 
        self.camera_info = msg


    def show_frame(self):
        
        if self.latest_frame is None:
                return
        
        frame = self.latest_frame.copy()


        detection_status = "apriltag not detected"
        if not self.camera_info is None:
            self.frame_width = self.camera_info.width
            self.frame_height = self.camera_info.height

            K = self.camera_info.k
            fx = K[0]
            fy = K[4]
            cx = K[2]
            cy = K[5]

            
            for det in self.detections:
                # draw bounding boxes
                corners = det.corners
                corners = [(int(c.x), int(c.y)) for c in corners]
                for i in range(4):
                    cv2.line(frame, corners[i], corners[(i+1)%4], (0, 0, 255), 5)

            
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
                
                L = 0.1 #axis length in meters
                pts_3d = np.array([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, L]])
                
                pts_cam = (R_mat @ pts_3d.T).T + t_vec

                # Project in 2D
                pts_2d = [] 
                for P in pts_cam:
                    X, Y, Z = P
                    u = int(fx*X/Z + cx)
                    v = int(fy*Y/Z + cy)
                    pts_2d.append((u, v))

                origin = pts_2d[0]
                cv2.line(frame, origin, pts_2d[1], (0, 0, 255), 4)
                cv2.line(frame, origin, pts_2d[2], (0, 255, 0), 4)
                cv2.line(frame, origin, pts_2d[3], (255, 0, 0), 4)

                # store detection status and position
                detection_status = f"apriltag detected at position x={t_vec[0]:.2f}, y={t_vec[1]:.2f}, z={t_vec[2]:.2f}"

    
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.7
        thickness = 2
        color = (0, 0, 255)

        # bottom-left corner text positions
        height, width, _ = frame.shape
        margin = 20
        y1 = margin 
        y2 = margin + 30

        cv2.putText(frame, f"detector: {self.detector_status}", (20, y1), font, scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, detection_status, (20, y2), font, scale, color, thickness, cv2.LINE_AA)

        cv2.imshow("Gimbal Camera Feed (SITL)", frame)
        cv2.waitKey(1) 

   
    def cleanup(self):
        """Clean up OpenCV windows before destroying the node."""
        cv2.destroyAllWindows()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
