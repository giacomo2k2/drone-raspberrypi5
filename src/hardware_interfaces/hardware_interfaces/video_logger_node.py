import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class LoggerNode(Node):

    def __init__(self):
        super().__init__('logger_node')

        # Create video folder
        video_folder = "Data/video"
        os.makedirs(video_folder, exist_ok=True)

        # Generate timestamped filename
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        self.video_path = os.path.join(video_folder, f"{timestamp}.mp4")

        # Open VideoWriter
        fourcc = cv2.VideoWriter_fourcc(*'avc1')  # H264/MP4
        fps = 30.0
        frame_size = (640, 480)
        self.video_writer = cv2.VideoWriter(self.video_path, fourcc, fps, frame_size)

        if not self.video_writer.isOpened():
            self.get_logger().error("Failed to open video file")
        else:
            self.get_logger().info(f"Recording video to: {self.video_path}")

        # CvBridge
        self.bridge = CvBridge()

        self.declare_parameter('camera_name', 'camera')
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            f'/{self.camera_name}/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Optional: resize if needed
            # frame = cv2.resize(frame, (640, 480))

            # Write frame to video
            self.video_writer.write(frame)

        except Exception as e:
            self.get_logger().error(f"CvBridge exception: {e}")

    def destroy_node(self):
        if self.video_writer.isOpened():
            self.video_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
