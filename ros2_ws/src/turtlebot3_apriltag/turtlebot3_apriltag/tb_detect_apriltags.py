import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pupil_apriltags

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.bridge = CvBridge()

        # Subscribe to TurtleBot3 camera feed
        self.image_sub = self.create_subscription(
            Image, '/image', self.image_callback, 10)

        # Publish processed image
        self.image_pub = self.create_publisher(Image, '/image_apriltag', 10)

        # Setup AprilTag detector
        self.at_detector = pupil_apriltags.Detector()

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect AprilTags
            tags = self.at_detector.detect(gray)
            for tag in tags:
                # Get the four corners of the tag
                pts = np.array(tag.corners, dtype=np.int32)
                cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)

                # Mark the ID
                cv2.putText(cv_image, str(tag.tag_id),
                            (int(tag.center[0]), int(tag.center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Convert back to ROS message and publish
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
