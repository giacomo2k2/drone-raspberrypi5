import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from drone_interfaces.srv import GenerateGlobalTrajectory  # replace with actual package/service
import numpy as np

# Import your trajectory functions here
from trajectory_planner.include.trajectory_planner.trajectory_utils import build_trajectory_normalized_3d, uniform_rescale_periods_3d_norm

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        self.srv = self.create_service(GenerateGlobalTrajectory, 'generate_trajectory', self.generate_callback)
        self.get_logger().info("Trajectory Generator Node started")

    def generate_callback(self, request, response):
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

            # Rescale periods to respect vmax/amax
            periods = uniform_rescale_periods_3d_norm(
                coeffs_x, coeffs_y, coeffs_z, periods_norm,
                request.vmax, request.amax
            )

            # Build response: coeffs[axis][segment][order+1]
            coeffs_list = []
            for axis_coeffs in coeffs_xyz:
                axis_list = [segment.tolist() for segment in axis_coeffs]
                coeffs_list.append(axis_list)

            response.coeffs = coeffs_list
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
