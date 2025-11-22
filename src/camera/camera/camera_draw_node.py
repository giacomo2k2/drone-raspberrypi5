#!/usr/bin/env python3
...

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image

class CameraDrawNode(Node):

    def __init__(self):
        super().__init__('camera_draw_node') # Renamed node to camera_draw_node to match launch file
        
        self.bridge = CvBridge()

 
        self.declare_parameter('camera_name', 'camera')
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        self.detections_image_sub = self.create_subscription(Image, f'/{self.camera_name}/image_detections_drawn', self.image_callback, 1)
        

        self.latest_frame = None
            
        cv2.namedWindow("Gimbal Camera Feed (SITL)", cv2.WINDOW_AUTOSIZE)

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.show_frame()

    def show_frame(self):
        
        if self.latest_frame is None:
                return
        
        frame = self.latest_frame.copy()


        cv2.imshow("Gimbal Camera Feed (SITL)", frame)
        cv2.waitKey(1) 

   
    def cleanup(self):
        """Clean up OpenCV windows before destroying the node."""
        cv2.destroyAllWindows()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraDrawNode()
    rclpy.spin(camera_node)
    camera_node.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
