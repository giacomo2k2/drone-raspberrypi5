import rclpy
from rclpy.node import Node
from drone_interfaces.srv import Shutdown, StopRecording
import subprocess
import threading


class ShutdownNode(Node):
    def __init__(self):
        super().__init__('shutdown_node')
        self.shutdown_service = self.create_service(
            Shutdown,
            '/drone/shutdown/shutdown_srv',
            self.shutdown_callback
        )

        self.stop_recording_client = self.create_client(StopRecording, 'drone/video_logger/stop_recording')
    

    def shutdown_callback(self, request, response):
        self.get_logger().info('Shutdown request received. Shutting down the system...')

        # First, stop video recording safely
        if not self.stop_recording_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Video Logger service not available. Cannot stop recording safely.')
        else:
            stop_request = StopRecording.Request()
            self.stop_recording_client.call_async(stop_request)
            
            
    
        try:
            # Send OS shutdown command
            subprocess.Popen(['sudo', 'shutdown', '-h', 'now'])
            response.success = True
            response.message = 'System is shutting down.'
            self.get_logger().info('Shutdown command sent to OS.')
        except Exception as e:
            response.success = False
            response.message = f'Failed to shutdown system: {e}'
            self.get_logger().error(f'Failed to shutdown system: {e}')

        # Schedule ROS shutdown asynchronously to avoid callback crashes
        threading.Thread(target=rclpy.shutdown).start()
        return response


def main(args=None):
    rclpy.init(args=args)
    shutdown_node = ShutdownNode()
    # Spin until shutdown_callback triggers rclpy.shutdown()
    rclpy.spin(shutdown_node)
    # After spin exits, destroy the node (no rclpy.shutdown() here)
    shutdown_node.destroy_node()


if __name__ == '__main__':
    main()

