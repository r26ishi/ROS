#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/my_camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # === Apply OpenCV image processing ===
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)

        # Show the result
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
