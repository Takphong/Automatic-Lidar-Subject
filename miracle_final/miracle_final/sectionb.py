#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Empty # Empty is for the docking signal
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class SectionBNode(Node):

    def __init__(self):
        super().__init__('section_b_node')
        
        # --- Section B: Navigation Components ---
        self.nav = BasicNavigator()
        self.nav_goal_sent = False
        
        # TODO: You MUST define your goal poses here 
        self.goal_poses = {
            1: (1.0, 0.5, 1.0),   # Example: ID 1 -> (x=1.0, y=0.5, w=1.0)
            2: (-0.5, 1.0, 0.707) # Example: ID 2 -> (x=-0.5, y=1.0, w=0.707)
            # Add all your possible ArUco Goal IDs and their map coordinates
        }
        
        # --- ROS2 Components ---
        # Subscriber to get goal ID from Section A
        self.goal_id_sub = self.create_subscription(
            Int16, '/goal_id', self.goal_id_callback, 10)
        
        # Publisher to trigger Section C
        self.docking_pub = self.create_publisher(Empty, '/docking_start', 10)
        
        self.goal_id = None
        self.nav.waitUntilNav2Active()
        self.get_logger().info('Section B node is active and waiting for Goal ID.')

    def goal_id_callback(self, msg):
        """ This is the main trigger for Section B """
        if self.nav_goal_sent: # Ensure we only navigate once
            return
            
        self.goal_id = msg.data
        self.get_logger().info(f'Received Goal ID: {self.goal_id}. Starting navigation.')
        
        # Get pose from our dictionary
        goal_pose_data = self.goal_poses.get(self.goal_id)
        
        if goal_pose_data is None:
            self.get_logger().error(f'No goal pose defined for ArUco ID {self.goal_id}!')
            return

        # Create PoseStamped message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_pose_data[0]
        goal_pose.pose.position.y = goal_pose_data[1]
        goal_pose.pose.orientation.w = goal_pose_data[2]
        
        # Send the goal
        self.nav.goToPose(goal_pose)
        self.nav_goal_sent = True
        
        # Create a timer to check for navigation completion
        self.check_timer = self.create_timer(1.0, self.check_nav_status)

    def check_nav_status(self):
        """ Check if navigation is complete """
        if not self.nav.isTaskComplete():
            return

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Section B: Navigation goal reached!')
            # Trigger Section C
            self.docking_pub.publish(Empty())
            self.get_logger().info('Published /docking_start signal. Shutting down.')
            
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Section B: Navigation was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Section B: Navigation failed!')
        
        # Stop timer and shut down node
        self.check_timer.cancel()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = SectionBNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
