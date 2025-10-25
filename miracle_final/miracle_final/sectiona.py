#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16 # To publish the Goal ID
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco
import time

# Define States for this node
STATE_FOLLOWING_PATH = 0
STATE_STOPPING_AT_QR = 1
STATE_TURNING = 2
STATE_WAITING_FOR_ARUCO = 3
STATE_DONE = 4

class SectionANode(Node):

    def __init__(self):
        super().__init__('section_a_node')
        
        # --- Parameters ---
        self.kp_angular = 0.005 
        self.linear_speed = 0.08
        self.angular_speed = 0.3 
        self.turn_duration_90_deg = 2.5 # TODO: You MUST tune this!
        
        # --- State Machine ---
        self.state = STATE_FOLLOWING_PATH
        self.get_logger().info('Section A node starting. State: FOLLOWING_PATH')

        # --- Variables ---
        self.line_error = 0.0
        self.turn_start_time = None
        
        # --- ROS2 Components ---
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher for communicating with Section B
        self.goal_id_pub = self.create_publisher(Int16, '/goal_id', 10)
        
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        self.control_loop_timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        
        # --- OpenCV Components ---
        self.qr_detector = cv2.QRCodeDetector()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = cv_image.shape
            
            if self.state == STATE_FOLLOWING_PATH:
                # --- Task A1: Path Following ---
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                # TODO: MUST tune these values
                lower_yellow = np.array([20, 100, 100])
                upper_yellow = np.array([30, 255, 255])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                search_top = int(3*h/4)
                search_bot = int(h)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0
                
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    image_center_x = w // 2
                    self.line_error = image_center_x - cx
                else:
                    self.line_error = 0
            
                # --- Task A2: QR Code Detection ---
                try:
                    detected, data, _, _ = self.qr_detector.detectAndDecode(cv_image)
                    if detected:
                        self.get_logger().info(f'QR Code detected! Data: {data}')
                        self.state = STATE_STOPPING_AT_QR
                        self.get_logger().info('State change: STOPPING_AT_QR')
                except Exception as e:
                    pass

            elif self.state == STATE_WAITING_FOR_ARUCO:
                # --- Task A2 Part 3: ArUco Detection [cite: 24, 33] ---
                corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
                if ids is not None:
                    goal_id = int(ids[0][0])
                    self.get_logger().info(f'ArUco marker detected! Goal ID: {goal_id}')
                    
                    # Publish the goal ID for Section B to hear
                    msg = Int16()
                    msg.data = goal_id
                    self.goal_id_pub.publish(msg)
                    
                    # Transition to final state
                    self.state = STATE_DONE
                    self.get_logger().info('State change: DONE. Publishing Goal ID and shutting down.')

        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')

    def control_loop(self):
        twist = Twist()

        if self.state == STATE_FOLLOWING_PATH:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.kp_angular * self.line_error
            
        elif self.state == STATE_STOPPING_AT_QR:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist) # Publish stop immediately
            
            self.turn_start_time = self.get_clock().now()
            self.state = STATE_TURNING
            self.get_logger().info('State change: TURNING')

        elif self.state == STATE_TURNING:
            elapsed_time = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            
            if elapsed_time < self.turn_duration_90_deg:
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed # Negative for right turn [cite: 23]
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = STATE_WAITING_FOR_ARUCO
                self.get_logger().info('State change: WAITING_FOR_ARUCO')
        
        elif self.state == STATE_WAITING_FOR_ARUCO:
            # Robot pauses and waits [cite: 24]
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        elif self.state == STATE_DONE:
            # Task complete, stop robot and shut down
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Section A complete. Shutting down node.')
            rclpy.shutdown()

        if self.state != STATE_DONE:
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SectionANode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
