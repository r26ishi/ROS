#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class CircleDetector(Node):
    def __init__(self):
        super().__init__('circle_detector')

        # Image subscriber
        self.subscription = self.create_subscription(
            Image,
            '/my_camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()

        # Circle position publisher
        self.publisher_ = self.create_publisher(Point, 'detected_circle', 10)

    def listener_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(
            gray_blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=100,
            param2=30,
            minRadius=10,
            maxRadius=100
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))

            # Only publish the first circle detected
            x, y, r = circles[0][0]

            # Draw the circle and center
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
            cv2.putText(frame, f"({x},{y})", (x-40, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # Publish circle center as Point
            point_msg = Point()
            point_msg.x = float(x)
            point_msg.y = float(y)
            point_msg.z = 0.0  # No depth from 2D image
            self.publisher_.publish(point_msg)

        cv2.imshow("Circle Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
