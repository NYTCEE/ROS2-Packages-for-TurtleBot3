import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import pupil_apriltags as apriltag

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower')
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image, '/image', self.image_callback, 10)

        # Publisher for TurtleBot3 movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("AprilTag Follower Node Started.")

    def image_callback(self, msg):
        # Convert ROS2 Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detections = self.detector.detect(gray)
        twist = Twist()

        if detections:
            tag = detections[0]  # Take the first detected tag
            cx, cy = tag.center  # Get center of the tag
            img_center_x = frame.shape[1] / 2  # Center of the image

            # Determine movement based on tag position
            if cx < img_center_x - 50:
                twist.angular.z = 0.3  # Turn left
            elif cx > img_center_x + 50:
                twist.angular.z = -0.3  # Turn right
            else:
                twist.angular.z = 0.0  # Centered
            
            twist.linear.x = 0.2  # Move forward
        else:
            twist.angular.z = 0.3  # Rotate to search for AprilTag

        # Publish movement command
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
