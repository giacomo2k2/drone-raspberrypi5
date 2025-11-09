import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class CameraCaptureNode (Node):
    def __init__(self):
        super().__init__('camera_capture_node')

        self.declare_parameter('camera_name', 'camera')
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        self.video_pub = self.create_publisher(Image, f'/{self.camera_name}/image_raw', 10)
        self.timer = self.create_timer(1/30, self.publish_video)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Adjust the camera index as needed
        self.bridge = CvBridge()

        
    def publish_video(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Cannot read frame")
            return
        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self.camera_name
        
        self.video_pub.publish(msg)


def main(args = None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
