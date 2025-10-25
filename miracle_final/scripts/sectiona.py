#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
# This is a newer way to import aruco in OpenCV
from cv2 import aruco
import time # For the timed 90-degree turn

# Define States
STATE_FOLLOWING_PATH = 0
STATE_STOPPING_AT_QR = 1
STATE_TURNING = 2
STATE_WAITING_FOR_ARUCO = 3
STATE_NAVIGATING_TO_GOAL = 4 # For Section B
STATE_DOCKING = 5            # For Section C

class FinalControllerNode(Node):

    def __init__(self):
        super().__init__('final_controller_node')
        
        # --- Parameters ---
        # PID gain for path following
        self.kp_angular = 0.005 
        # Target speed
        self.linear_speed = 0.08
        # Speed for turning
        self.angular_speed = 0.3 
        # Duration for a 90-degree turn (NEEDS CALIBRATION on the real robot)
        self.turn_duration_90_deg = 2.5 # You MUST tune this!
        
        # --- State Machine ---
        self.state = STATE_FOLLOWING_PATH
        self.get_logger().info(f'Initializing. Current state: {self.state}')

        # --- Variables ---
        self.line_error = 0.0
        self.turn_start_time = None
        self.goal_id = None # To store the ArUco ID
        
        # --- ROS2 Components ---
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # This subscriber is the main "engine" of the node
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust topic if needed (e.g., /image_raw)
            self.image_callback,
            10)
        
        # This timer is the "action" part of the node
        self.control_loop_timer = self.create_timer(0.05, self.control_loop) # 20 Hz loop
        
        # --- OpenCV Components ---
        self.qr_detector = cv2.QRCodeDetector()
        
        # Initialize ArUco detector
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50) # Example dict
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.get_logger().info('Final Controller Node is running.')

    def image_callback(self, msg):
        """
        This function receives the image and performs ALL vision processing.
        It sets variables (like self.line_error) and changes the state.
        It DOES NOT publish velocity commands.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = cv_image.shape
            
            # --- State-based Image Processing ---
            
            if self.state == STATE_FOLLOWING_PATH:
                # --- Task A1: Path Following  ---
                
                # 1. Convert to HSV color space
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                
                # 2. Define color range for YELLOW
                #    You MUST tune these values for the real arena lighting
                lower_yellow = np.array([20, 100, 100])
                upper_yellow = np.array([30, 255, 255])
                
                # 3. Create a binary mask
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                # 4. Find the centroid of the yellow line
                #    We only care about the bottom half of the image
                h, w, d = cv_image.shape
                search_top = int(3*h/4)
                search_bot = int(h)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0
                
                # 5. Calculate moments
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    # Calculate X coordinate of the centroid
                    cx = int(M['m10'] / M['m00'])
                    # Calculate Y coordinate (optional, for debugging)
                    # cy = int(M['m01'] / M['m00'])
                    # cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # 6. Calculate error
                    #    Error = center_of_image - center_of_line
                    image_center_x = w // 2
                    self.line_error = image_center_x - cx
                else:
                    # No line detected, maybe stop or keep last error
                    self.line_error = 0 # Or you could set a "lost line" flag
            
                # --- Task A2: QR Code Detection  ---
                #    We do this at the same time as path following
                try:
                    detected, data, _, _ = self.qr_detector.detectAndDecode(cv_image)
                    if detected:
                        # The QR code is the "STOP sign" 
                        self.get_logger().info(f'QR Code detected! Data: {data}')
                        # Transition to the next state
                        self.state = STATE_STOPPING_AT_QR
                        self.get_logger().info(f'State change: {self.state}')
                except Exception as e:
                    pass # QR detection can be noisy

            elif self.state == STATE_WAITING_FOR_ARUCO:
                # --- Task A2 Part 3: ArUco Detection [cite: 24, 33] ---
                corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
                if ids is not None:
                    # We found a marker!
                    self.goal_id = int(ids[0][0]) # Get the first ID found
                    self.get_logger().info(f'ArUco marker detected! Goal ID: {self.goal_id}')
                    
                    # Transition to Section B
                    self.state = STATE_NAVIGATING_TO_GOAL
                    self.get_logger().info(f'State change: {self.state}')
                    
                    # Store this goal_id to use in Section B's navigation [cite: 38]

            # (Optional) Display the image for debugging
            # cv2.imshow("Camera View", cv_image)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')

    def control_loop(self):
        """
        This function runs on a timer (20Hz).
        It reads the current state and variables (like self.line_error)
        and publishes the appropriate Twist command.
        """
        twist = Twist()

        if self.state == STATE_FOLLOWING_PATH:
            # --- Task A1: Path Following  ---
            # Simple P-controller [cite: 30]
            twist.linear.x = self.linear_speed
            twist.angular.z = self.kp_angular * self.line_error
            
        elif self.state == STATE_STOPPING_AT_QR:
            # --- Task A2: Stop at QR  ---
            # Stop the robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist) # Publish stop command immediately
            
            # Log the start time for the turn
            self.turn_start_time = self.get_clock().now()
            # Transition to the next state
            self.state = STATE_TURNING
            self.get_logger().info(f'State change: {self.state}')

        elif self.state == STATE_TURNING:
            # --- Task A2: Turn 90 Degrees Right [cite: 23, 32] ---
            # This is an open-loop (timed) turn.
            # A closed-loop turn using /odom or /imu is more robust
            # but more complex.
            
            elapsed_time = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            
            if elapsed_time < self.turn_duration_90_deg:
                # Turning right
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed # Negative for right turn
            else:
                # Turn complete
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
                # Transition to the next state
                self.state = STATE_WAITING_FOR_ARUCO
                self.get_logger().info(f'State change: {self.state}')
        
        elif self.state == STATE_WAITING_FOR_ARUCO:
            # --- Task A2: Pause and Wait  ---
            # The robot must pause and wait.
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        elif self.state == STATE_NAVIGATING_TO_GOAL:
            # --- Section B ---
            # Stop for now. Logic for navigation will go here.
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # (Your navigation logic from Section B will take over)
        
        elif self.state == STATE_DOCKING:
            # --- Section C ---
            # Stop for now. Logic for docking will go here.
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # (Your docking logic from Section C will take over)

        # Publish the command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FinalControllerNode()
    rclpy.spin(node)
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
