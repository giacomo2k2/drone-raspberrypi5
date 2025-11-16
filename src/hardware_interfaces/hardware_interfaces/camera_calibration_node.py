import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
import math
import yaml
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ament_index_python.packages import get_package_share_directory

from drone_interfaces.srv import ActivateDetector

import cv2
import os

from std_msgs.msg import Bool

class CameraCalibrationNode(Node): 
    def __init__(self): 
        super().__init__('camera_calibration_node')

        package_share = get_package_share_directory('hardware_interfaces')


        self.declare_parameter('platform', "CC")

        self.platform = self.get_parameter('platform').value

        self.declare_parameter('camera_name', 'camera')
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        if self.platform == "SITL":
            config_file = os.path.join(package_share, 'config', 'sitl_camera_configuration.yaml')
        else:
            config_file = os.path.join(package_share, 'config', 'cc_camera_configuration.yaml')
               # publisher
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        with open(config_file, 'r') as f:
            cfg = yaml.safe_load(f)

        if self.platform == "SITL":

            self.image_width = cfg["camera_calibration_node"]["ros__parameters"]["image_width"]
            self.image_height = cfg["camera_calibration_node"]["ros__parameters"]["image_height"]
            self.horizontal_fov = cfg["camera_calibration_node"]["ros__parameters"]['horizontal_fov']

            cx = self.image_width/2.0
            cy = self.image_height/2.0

            fx = fy = self.image_width/(2*math.tan(self.horizontal_fov/2))

            mtx = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0, 0, 1]])
            dist = np.zeros([5])

             # subscriptions
            self.video_sub = self.create_subscription(Image, f'/{self.camera_name}/image_raw', self.get_published_frame, qos_profile) 

        else:
            self.image_width = cfg["camera_calibration_node"]["ros__parameters"]["image_width"]
            self.image_height = cfg["camera_calibration_node"]["ros__parameters"]["image_height"]
            mtx = np.array(cfg["camera_calibration_node"]["ros__parameters"]['camera_matrix']['data']).reshape((3,3))
            dist = np.array(cfg["camera_calibration_node"]["ros__parameters"]['distortion_coefficients']['data'])


            self.video_raw_pub = self.create_publisher(Image, f'/{self.camera_name}/image_raw', 10)
            self.timer = self.create_timer(1.0/30.0, self.get_frame)
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Adjust the camera index as needed

        self.bridge = CvBridge()

        self.info_pub = self.create_publisher(CameraInfo, f'/{self.camera_name}/camera_info', qos_profile)
        self.video_pub = self.create_publisher(Image, f'/{self.camera_name}/image_rect', qos_profile)
        self.detector_status_pub = self.create_publisher(Bool, '/drone/camera_calibration/detector_status', 10)
        
       
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
        

        self.timer_detector = self.create_timer(1.0, self.publish_detector_status)

    def publish_detector_status(self):
        msg = Bool()
        msg.data = self.detector_active

        self.detector_status_pub.publish(msg)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Cannot read frame")
            return
        image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        self.video_raw_pub.publish(image)
        self.process_and_publish_cb(frame)

    def get_published_frame(self, raw_image_msg: Image):
        frame = self.bridge.imgmsg_to_cv2(raw_image_msg, desired_encoding='bgr8')
        self.process_and_publish_cb(frame)

    def activate_detector_cb(self, request, response):
        response.success = True


        if request.activate:
            self.detector_active = True
            self.get_logger().info("Camera calibration detector activated via service call.")
        else:
            self.detector_active = False
            self.get_logger().info("Camera calibration detector deactivated via service call.")
        
        return response

    
    def process_and_publish_cb(self, cv_image: np.ndarray):    

        if not self.detector_active:
            return

        rect_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        
        now = self.get_clock().now().to_msg()

        rect_image_msg.header.stamp = now
        self.camera_info.header.stamp = now
        rect_image_msg.header.frame_id = self.camera_name
        self.camera_info.header.frame_id = self.camera_name

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
