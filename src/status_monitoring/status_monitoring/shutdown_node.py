import rclpy

from rclpy.node import Node

from drone_interfaces.srv import Shutdown

import os
import subprocess


class ShutdownNode(Node):
    def __init__(self):
        super().__init__('shutdown_node')
        
        self.shutdown_service = self.create_service(Shutdown, '/drone/shutdown/shutdown_srv', self.shutdown_callback)


    def shutdown_callback(self, request, response):
        self.get_logger().info('Shutdown request received. Shutting down the system...')

        # Stop ROS 2 nodes/services (optional: you can also just rclpy.shutdown())
        # If you want to shutdown the ROS system gracefully:
        rclpy.shutdown()
        self.get_logger().info('ROS 2 shutdown complete.')

        # Optional: any pre-shutdown tasks here
        # e.g., stop drone motors, save logs, etc.

        # Shutdown the Linux system (Companion Computer)
        try:
            # Using subprocess for sudo shutdown
            subprocess.Popen(['sudo', 'shutdown', '-h', 'now'])
            response.success = True
            response.message = 'System is shutting down.'
            self.get_logger().info('Shutdown command sent to OS.')
        except Exception as e:
            response.success = False
            response.message = f'Failed to shutdown system: {e}'
            self.get_logger().error(f'Failed to shutdown system: {e}')

        return response

def main(args=None):
    rclpy.init(args=args)
    shutdown_node = ShutdownNode()
    rclpy.spin(shutdown_node)
    shutdown_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()