#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- Define States ---
STATE_FOLLOWING_PATH = 0
STATE_STOPPING_AT_QR = 1
STATE_TURNING = 2
STATE_WAITING_FOR_ARUCO = 3
STATE_NAVIGATING_TO_GOAL = 4
STATE_DOCKING = 5
STATE_TASK_COMPLETE = 6

class AllInOneNode(Node):

    def __init__(self):
        super().__init__('allinone_node')

        # --- Parameters ---
        self.kp_angular_path = 0.005
        self.linear_speed_path = 0.08
        self.angular_speed_turn = 0.3
        self.turn_duration_90_deg = 6.1 #normally it's 2.5 but my PC said no and ignore to turn 90 deg :(

        # Docking parameters
        self.kp_angular_dock = 0.015
        self.kp_linear_dock = 0.3
        self.docking_target_dist = 0.10
        self.docking_dist_tolerance = 0.01
        self.docking_ang_tolerance_px = 10
        self.max_linear_dock = 0.05
        self.max_angular_dock = 0.1

        # --- State Machine ---
        self.state = STATE_FOLLOWING_PATH
        self.get_logger().info(f'Starting. Current state: {self.state}')

        # --- Variables ---
        self.line_error = 0.0
        self.turn_start_time = None
        self.goal_id = None
        self.docking_x_error_px = 0.0
        self.docking_dist_error_m = 0.0
        self.last_marker_seen_time = None
        self.nav_goal_sent = False

        # --- ROS Interfaces ---
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/oak/rgb/camera/image_raw/compressed', self.image_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        # --- Nav2 (Headless Startup) ---
        self.nav = BasicNavigator()
        self.get_logger().info('Waiting for Nav2 lifecycle to become active (no RViz)...')
        try:
            self.nav.waitUntilNav2Active(timeout_sec=30)
            self.get_logger().info('✅ Nav2 is active and ready (headless mode).')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Nav2 not confirmed active: {e}')

        # --- ArUco / QR ---
        self.qr_detector = cv2.QRCodeDetector()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # --- Camera calibration (placeholder, replace with real) ---
        self.camera_matrix = np.array([[0.0, 0.0, 0.0],
                                       [0.0, 0.0, 0.0],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros(5)
        self.aruco_marker_size = 0.05

        # --- Goal Map (Aruco IDs to goals) ---
        self.goal_poses = {
            1: (3.106, -0.105, 0.362),
            2: (3.106, -0.620, 1.0),
            3: (3.106, -0.894, 1.0),
            4: (3.106, -1.286, 1.0),
            5: (3.106, -1.655, 1.0)
        }

        self.get_logger().info('All-in-one headless controller is running.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = cv_image.shape
            center_x = w // 2

            if self.state == STATE_FOLLOWING_PATH:
                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                lower_yellow = np.array([25, 50, 180])
                upper_yellow = np.array([40, 255, 255])
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

                search_top, search_bot = int(3*h/4), int(h)
                mask[0:search_top, :] = 0
                mask[search_bot:h, :] = 0

                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    self.line_error = center_x - cx
                else:
                    self.get_logger().warn('No yellow line detected. Stopping.')
                    self.state = STATE_STOPPING_AT_QR

                # Concurrent QR detection
                detected, data, _, _ = self.qr_detector.detectAndDecode(cv_image)
                if detected:
                    self.get_logger().info(f'QR detected: {data}')
                    self.state = STATE_STOPPING_AT_QR

            elif self.state == STATE_WAITING_FOR_ARUCO:
                corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
                if ids is not None:
                    self.goal_id = int(ids[0][0])
                    self.get_logger().info(f'ArUco marker ID {self.goal_id} detected.')
                    self.state = STATE_NAVIGATING_TO_GOAL

            elif self.state == STATE_DOCKING:
                corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
                if ids is not None:
                    target_corners = corners[0]
                    marker_center_x = np.mean(target_corners[0][:, 0])
                    self.docking_x_error_px = center_x - marker_center_x

                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        target_corners, self.aruco_marker_size,
                        self.camera_matrix, self.dist_coeffs)
                    distance = tvecs[0][0][2]
                    self.docking_dist_error_m = distance - self.docking_target_dist
                    self.last_marker_seen_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')

    def control_loop(self):
        twist = Twist()

        if self.state == STATE_FOLLOWING_PATH:
            twist.linear.x = self.linear_speed_path
            twist.angular.z = self.kp_angular_path * self.line_error

        elif self.state == STATE_STOPPING_AT_QR:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.turn_start_time = self.get_clock().now()
            self.state = STATE_TURNING
            self.get_logger().info('Turning 90°...')

        elif self.state == STATE_TURNING:
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
            if elapsed < self.turn_duration_90_deg:
                twist.angular.z = -self.angular_speed_turn
            else:
                twist.angular.z = 0.0
                self.state = STATE_WAITING_FOR_ARUCO
                self.get_logger().info('Turn complete. Waiting for ArUco marker.')

        elif self.state == STATE_NAVIGATING_TO_GOAL:
            if not self.nav_goal_sent:
                self.nav_goal_sent = True
                pose_data = self.goal_poses.get(self.goal_id)
                if pose_data is None:
                    self.get_logger().error(f'No goal defined for ID {self.goal_id}')
                    return

                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self.nav.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w = pose_data
                self.nav.goToPose(goal)
                self.get_logger().info(f'Navigating to goal {self.goal_id}...')

        #elif self.state == STATE_NAVIGATING_TO_GOAL: # I might use this part if I don't have enough time :(
            ## Skip Nav2 for now — simple forward movement placeholder
            #twist.linear.x = 0.1
            #twist.angular.z = 0.0
            #self.cmd_vel_pub.publish(twist)
            #time.sleep(2.0)  # simulate moving toward goal
            #twist.linear.x = 0.0
            #self.cmd_vel_pub.publish(twist)
            #self.get_logger().info('Simulated navigation complete. Proceeding to docking.')
            #self.state = STATE_DOCKING


            if self.nav.isTaskComplete():
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('Navigation complete. Proceeding to docking.')
                    self.state = STATE_DOCKING
                else:
                    self.get_logger().warn(f'Navigation result: {result}')

        elif self.state == STATE_DOCKING:
            if abs(self.docking_dist_error_m) < self.docking_dist_tolerance and \
               abs(self.docking_x_error_px) < self.docking_ang_tolerance_px:
                self.get_logger().info('✅ Docking complete.')
                self.state = STATE_TASK_COMPLETE
            else:
                lin = np.clip(self.kp_linear_dock * self.docking_dist_error_m,
                              -self.max_linear_dock, self.max_linear_dock)
                ang = np.clip(-self.kp_angular_dock * self.docking_x_error_px,
                              -self.max_angular_dock, self.max_angular_dock)
                twist.linear.x = lin
                twist.angular.z = ang

        elif self.state == STATE_TASK_COMPLETE:
            twist.linear.x = twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    node = AllInOneNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
