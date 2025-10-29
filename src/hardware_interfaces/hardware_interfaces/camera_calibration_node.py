import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import math
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from drone_interfaces.srv import ActivateDetector

from std_msgs.msg import Bool

CAMERA_NAME = 'gimbal_camera'

class CameraCalibrationNode(Node): 
    def __init__(self): 
        super().__init__('camera_calibration_node')

        self.image_width = 640
        self.image_height = 480
        self.horizontal_fov = 2.0

        cx = self.image_width/2.0
        cy = self.image_height/2.0

        fx = fy = self.image_width/(2*math.tan(self.horizontal_fov/2))

        mtx = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0, 0, 1]])
        dist = np.zeros([5])

        self.bridge = CvBridge()

        # publisher

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.info_pub = self.create_publisher(CameraInfo, f'/{CAMERA_NAME}/camera_info', qos_profile)
        self.video_pub = self.create_publisher(Image, f'/{CAMERA_NAME}/image_rect', qos_profile)
        self.detector_status_pub = self.create_publisher(Bool, '/drone/camera_calibration/detector_status', 10)
        
        self.video_sub = self.create_subscription(
            Image,
            f'/{CAMERA_NAME}/image_raw',
            self.process_and_publish_cb,
            1) 
        

        self.activate_detector_srv = self.create_service(ActivateDetector, '/drone/camera_calibration/activate_detector', self.activate_detector_cb)

        self.detector_active = False

        # prepare camera info message


        self.camera_info = CameraInfo()
        self.camera_info.width = self.image_width
        self.camera_info.height = self.image_height
        self.camera_info.k = mtx.flatten().tolist()
        self.camera_info.d = dist.flatten().tolist()
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.r = np.eye(3).flatten().tolist()
        self.camera_info.p = np.hstack((mtx, np.zeros((3, 1)))).flatten().tolist()

        self.timer = self.create_timer(1.0, self.publish_detector_status)

    def publish_detector_status(self):
        msg = Bool()
        msg.data = self.detector_active

        self.detector_status_pub.publish(msg)

    def activate_detector_cb(self, request, response):
        response.success = True


        if request.activate:
            self.detector_active = True
            self.get_logger().info("Camera calibration detector activated via service call.")
        else:
            self.detector_active = False
            self.get_logger().info("Camera calibration detector deactivated via service call.")
        
        return response



    def process_and_publish_cb(self, raw_image_msg: Image):    

        if not self.detector_active:
            return

        cv_image = self.bridge.imgmsg_to_cv2(raw_image_msg, desired_encoding='bgr8')

        rect_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        
        
        rect_image_msg.header = raw_image_msg.header
        self.camera_info.header = raw_image_msg.header
        rect_image_msg.header.frame_id = CAMERA_NAME
        self.camera_info.header.frame_id = CAMERA_NAME

        self.video_pub.publish(rect_image_msg)
        self.info_pub.publish(self.camera_info)
        

   

def main(args = None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()