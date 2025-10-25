#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty # To get signal from Section B
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco

class SectionCNode(Node):

    def __init__(self):
        super().__init__('section_c_node')
        
        # --- Parameters (Section C) ---
        self.kp_angular_dock = 0.8  # TODO: MUST TUNE
        self.kp_linear_dock = 0.3   # TODO: MUST TUNE
        self.docking_target_dist = 0.10 # 10 cm target 
        self.docking_dist_tolerance = 0.01 # 1 cm tolerance [cite: 52]
        self.docking_ang_tolerance_px = 5 # Pixel tolerance for centering
        self.max_linear_dock = 0.05
        self.max_angular_dock = 0.1
        
        # --- State ---
        self.is_active = False # Becomes true when we get the /docking_start signal
        
        # --- Variables ---
        self.x_error = 0.0
        self.dist_error = 0.0
        
        # --- ROS2 Components ---
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to trigger this node
        self.start_sub = self.create_subscription(
            Empty, '/docking_start', self.start_callback, 10)
            
        self.image_sub = None # We will create this only when active
        self.control_loop_timer = None # We will create this only when active

        # --- OpenCV Components ---
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # --- Camera Calibration (REQUIRED) ---
        # TODO: You MUST provide your camera's calibration matrix and distortion
        self.camera_matrix = np.array([
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32) # Fill with your K matrix
        
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        
        self.aruco_marker_size = 0.05 # TODO: MUST TUNE (in meters)

        self.get_logger().info('Section C node is active and waiting for /docking_start signal.')

    def start_callback(self, msg):
        """ This is the main trigger for Section C """
        if self.is_active:
            return
            
        self.get_logger().info('Received /docking_start signal. Activating Docking.')
        self.is_active = True
        
        # Now we start listening to the camera and controlling the robot
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.control_loop_timer = self.create_timer(0.05, self.control_loop)

    def image_callback(self, msg):
        if not self.is_active:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = cv_image.shape
            image_center_x = w // 2
            
            corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
            
            if ids is not None:
                # Assuming the first marker found is the one we want to dock to
                target_corners = corners[0]
                
                # --- 1. Angular Error (Centering) ---
                # Get the center of the ArUco marker
                marker_center_x = np.mean(target_corners[0][:, 0])
                self.x_error = image_center_x - marker_center_x # In pixels
                
                # --- 2. Distance Error ---
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    target_corners, 
                    self.aruco_marker_size, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # tvecs[0][0][2] is the Z distance
                distance = tvecs[0][0][2]
                self.dist_error = distance - self.docking_target_dist
            else:
                # No marker visible, stop
                self.x_error = 0.0
                self.dist_error = 0.0
                self.get_logger().warn('Docking: ArUco marker not visible.', throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')

    def control_loop(self):
        if not self.is_active:
            return
            
        twist = Twist()
        
        # Check if we are at the goal
        if abs(self.dist_error) < self.docking_dist_tolerance and \
           abs(self.x_error) < self.docking_ang_tolerance_px:
            
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            self.get_logger().info('Section C: Docking complete! Shutting down.')
            self.is_active = False
            rclpy.shutdown()
            
        else:
            # --- Proportional Control for Docking ---
            
            # Linear control (distance)
            lin_vel = self.kp_linear_dock * self.dist_error
            
            # Angular control (centering)
            # Note: kP for angular is tuned based on pixel error
            ang_vel = (self.kp_angular_dock / 100.0) * self.x_error # Simple P-control on pixels

            # Clamp (limit) speeds to be safe
            twist.linear.x = np.clip(lin_vel, -self.max_linear_dock, self.max_linear_dock)
            twist.angular.z = np.clip(ang_vel, -self.max_angular_dock, self.max_angular_dock)
            
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SectionCNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
