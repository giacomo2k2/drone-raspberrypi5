#!/usr/bin/env python3

import cv2
import numpy as np
import yaml
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class CameraCalibrationNode(Node): 
    def __init__(self):
        super().__init__('camera_calibration_node')


        # Chessboard parameters
        self.chessboard_size = (9, 6)   # inner corners (cols, raws)
        self.square_size = 0.023        # meters

        # Prepare object points
        self.objp = np.zeros((self.chessboard_size[1]*self.chessboard_size[0], 3), np.float32)                  # all coordinates of corners in the plane
        self.objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)      # assigns coordinates to the corners in the image (z = 0)  
        
        # Storage for points
        self.objpoints = [] # 3D points in world space
        self.imgpoints = [] # 2D points in image place


        # open webcam
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open webcam")
            exit(1)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))

        self.get_logger().info("Press SPACE when chessboard is detected to capture frame, ESC to finish")

        self.calibrate()


    def calibrate(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_cb, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

            display = frame.copy()
            if ret_cb:
                cv2.drawChessboardCorners(display, self.chessboard_size, corners, ret_cb)

            cv2.imshow('Calibration', display)
            key = cv2.waitKey(1) & 0xFF

            if key == 27: # ESC
                break
            elif key == 32 and ret_cb: # SPACE
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                self.imgpoints.append(corners2)
                self.objpoints.append(self.objp)
                self.get_logger().info(f"Captured frame #{len(self.imgpoints)}")

        self.cap.release()
        cv2.destroyAllWindows()
        self.compute_calibration(gray.shape[::-1])


    def compute_calibration(self, image_size):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, image_size, None, None)
        self.get_logger().info(f"Calibration done, reprojection error: {self.compute_reprojection_error(mtx, dist, rvecs, tvecs, image_size):.4f}")

        # Save YAML for Ros2 camera_info_publisher
        pkg_path = get_package_share_directory('drone_config')
        config_dir = os.path.join(pkg_path, 'config')
        os.makedirs(config_dir, exist_ok=True)
        file_path = os.path.join(config_dir, 'camera.yaml')

        data = {
            'image_width': image_size[0],
            'image_height': image_size[1],
            'camera_name': 'camera',
            'camera_matrix': {
                'rows': 3, 'cols': 3,
                'data': mtx.flatten().tolist()
            },
            'distortion_model': 'plumb_bob',
            'distortion_coefficients': {
                'rows': 1,
                'cols': len(dist.flatten()),
                'data': dist.flatten().tolist()
            },
            'rectification_matrix': {
                'rows': 3, 'cols': 3,
                'data': np.eye(3).flatten().tolist()
            },
            'projection_matrix': {
                'rows': 3, 'cols': 4,
                'data': np.hstack((mtx, np.zeros((3, 1)))).reshape(-1).tolist()
            }
        }


        with open(file_path, 'w') as f:
            yaml.dump(data, f)

        self.get_logger().info(f"Calibration saved to {file_path}")


    def compute_reprojection_error(self, mtx, dist, rvecs, tvecs, image_size):
        total_error = 0

        for i in range(len(self.objpoints)):
            imgpoints2, _ = cv2.projectPoints(self.objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(self.imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            total_error += error

        return total_error/len(self.objpoints)


def main(args = None):
    rclpy.init(args = args)
    node = CameraCalibrationNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()