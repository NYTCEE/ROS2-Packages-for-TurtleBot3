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

        # 訂閱 TurtleBot3 的攝影機影像
        self.image_sub = self.create_subscription(
            Image, '/image', self.image_callback, 10)

        # 發布處理後的影像
        self.image_pub = self.create_publisher(Image, '/image_apriltag', 10)

        # 設定 AprilTag 偵測器
        self.at_detector = pupil_apriltags.Detector()

    def image_callback(self, msg):
        try:
            # 轉換 ROS 影像訊息到 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 轉成灰階
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # 偵測 AprilTag
            tags = self.at_detector.detect(gray)
            for tag in tags:
                # 取得標籤的四個角點
                pts = np.array(tag.corners, dtype=np.int32)
                cv2.polylines(cv_image, [pts], True, (0, 255, 0), 2)

                # 標記 ID
                cv2.putText(cv_image, str(tag.tag_id),
                            (int(tag.center[0]), int(tag.center[1])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # 轉回 ROS 訊息並發布
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"影像處理錯誤: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
