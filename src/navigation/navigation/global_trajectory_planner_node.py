#!/usr/bin/env python3
...

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from drone_interfaces.srv import GenerateGlobalTrajectory  


import numpy as np


from trajectory_utils import build_trajectory_normalized_3d, uniform_rescale_periods_3d_norm

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')

        self.srv = self.create_service(GenerateGlobalTrajectory, 'drone/global_trajectory_planner/generate_global_trajectory', self.generate_trajectory_callback)

    def generate_trajectory_callback(self, request, response):
        try:
            # Convert waypoints from geometry_msgs/Point to np.array
            waypoints = np.array([[p.x, p.y, p.z] for p in request.waypoints])
            
            # Generate trajectory coefficients
            coeffs_xyz, periods_norm = build_trajectory_normalized_3d(
                waypoints,
                request.vmax,
                request.amax
            )

            # Split coefficients per axis
            coeffs_x = coeffs_xyz[0, :, :]
            coeffs_y = coeffs_xyz[1, :, :]
            coeffs_z = coeffs_xyz[2, :, :]

            # self.get_logger().info(f"x_coeffs: {coeffs_x}")
            # self.get_logger().info(f"y_coeffs: {coeffs_y}")
            # self.get_logger().info(f"z_coeffs: {coeffs_z}")    
        

            # Rescale periods to respect vmax/am    ax
            periods = uniform_rescale_periods_3d_norm(
                coeffs_x, coeffs_y, coeffs_z, periods_norm,
                request.vmax, request.amax
            )

            # Fill response

            response.coeffs_x = coeffs_x.flatten().tolist()
            response.coeffs_y = coeffs_y.flatten().tolist()
            response.coeffs_z = coeffs_z.flatten().tolist()

            response.num_segments = coeffs_x.shape[0]
            response.num_coeffs_segment = coeffs_x.shape[1]

            response.periods = periods.tolist()

            response.success = True
            response.message = "Trajectory generated successfully"

        except Exception as e:
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
