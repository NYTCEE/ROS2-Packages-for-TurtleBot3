import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import pupil_apriltags as apriltag
import time

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower')
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image, '/image', self.image_callback, 10)

        # Publish motion commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
       
        # Search mode parameters
        self.search_state = 0  # Search state
        self.search_states = [
            {'angle': 60, 'direction': 1, 'speed': 0.5},    # Small left half-circle
            {'angle': 120, 'direction': -1, 'speed': 0.5},  # Return to center
            {'angle': 120, 'direction': 1, 'speed': 0.5},   # Small right half-circle
            {'angle': 180, 'direction': -1, 'speed': 0.7},  # Medium left half-circle
            {'angle': 180, 'direction': 1, 'speed': 0.7},   # Return to center
            {'angle': 240, 'direction': -1, 'speed': 0.7},  # Medium right half-circle
        ]
        self.start_time = self.get_clock().now()
        self.target_reached = False
       
        # Timer to periodically check search state
        self.timer = self.create_timer(0.1, self.search_timer_callback)

        self.get_logger().info("AprilTag Follower Node Started.")

    def search_timer_callback(self):
        """Periodically check and update search state"""
        if self.target_reached:
            # If target angle is reached, switch to next state
            self.search_state = (self.search_state + 1) % len(self.search_states)
            self.start_time = self.get_clock().now()
            self.target_reached = False
            self.get_logger().info(f"Switching to search state {self.search_state}")

    def image_callback(self, msg):
        # Convert ROS2 image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = self.detector.detect(gray)
        twist = Twist()

        if detections:
            # AprilTag found, switch to tracking mode
            tag = detections[0]  # Take the first detected tag
            cx, cy = tag.center  # Get tag center coordinates
            img_center_x = frame.shape[1] / 2  # Image center point (x-axis)

            # Use proportional control to adjust angular velocity
            error_x = cx - img_center_x  # Calculate horizontal offset of the tag
            twist.angular.z = 0.002 * error_x  # Adjust turning (P control)

            # Change TurtleBot linear velocity based on tag distance
            tag_size = tag.corners[1][0] - tag.corners[0][0]  # Tag width (approximate estimate)
            if tag_size > 150:  
                twist.linear.x = 0.0  # Stop if too close
            else:
                twist.linear.x = 0.2  # Otherwise move forward
               
            # Reset search state
            self.search_state = 0
            self.start_time = self.get_clock().now()
            self.target_reached = False
       
        else:
            # No AprilTag found, execute search pattern
            current_state = self.search_states[self.search_state]
           
            # Calculate rotation time elapsed
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
           
            # Calculate time needed for target angle (angle/angular_velocity)
            # Angular velocity is current_state['speed'], time = angle / angular_velocity
            target_time = abs(current_state['angle'] * 3.14159 / 180.0 / current_state['speed'])
           
            if elapsed < target_time and not self.target_reached:
                # Target angle not reached, continue rotating
                twist.angular.z = current_state['speed'] * current_state['direction']
                self.get_logger().debug(f"Searching: state {self.search_state}, angle {current_state['angle']}, elapsed {elapsed:.2f}/{target_time:.2f}")
            else:
                # Target angle reached
                twist.angular.z = 0.0
                self.target_reached = True

        # Send velocity command
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
