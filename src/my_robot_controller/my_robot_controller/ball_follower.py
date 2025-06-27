#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class CircleFollower(Node):
    def __init__(self):
        super().__init__('circle_follower')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust topic as needed
            self.image_callback,
            10)
        self.br = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_width = 640  # update to match your camera resolution
        self.dead_zone = 50     # tolerance for "centered" ball

    def image_callback(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (9, 9), 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=100,
            param2=30,
            minRadius=10,
            maxRadius=100
        )

        twist = Twist()

        if circles is not None:
            circles = np.uint16(np.around(circles))
            x, y, r = circles[0][0]  # first circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
            cv2.putText(frame, f"({x},{y})", (x-40, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # Decision logic
            error_x = x - self.image_width // 2

            if abs(error_x) < self.dead_zone:
                twist.linear.x = 0.2  # Move forward
                twist.angular.z = 0.0
            elif error_x < 0:
                twist.linear.x = 0.0
                twist.angular.z = 0.2  # Turn left
            else:
                twist.linear.x = 0.0
                twist.angular.z = -0.2  # Turn right

            self.cmd_pub.publish(twist)

        else:
            # Stop if no ball is found
            self.cmd_pub.publish(Twist())

        cv2.imshow("Follower", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CircleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
