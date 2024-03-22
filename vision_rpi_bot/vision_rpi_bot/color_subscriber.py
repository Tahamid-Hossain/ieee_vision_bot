# your_package/your_package/color_subscriber.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    hue = hsvC[0][0][0]  # Get the hue value
    if color == [0, 0, 255]:  # Red in BGR
        return [(np.array([0, 100, 100], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
                (np.array([160, 100, 100], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))]
    elif color == [255, 0, 0]:  # Blue in BGR
        return [(np.array([hue - 10, 100, 100], dtype=np.uint8), np.array([hue + 10, 255, 255], dtype=np.uint8))]
    return [(np.array([hue - 10, 100, 100], dtype=np.uint8), np.array([hue + 10, 255, 255], dtype=np.uint8))]

class ColorSubscriber(Node):
    def __init__(self):
        super().__init__('color_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Red and Blue detection
        for color, color_name in ( ([0, 0, 255], "Red"), ([255, 0, 0], "Blue") ):
            for lowerLimit, upperLimit in get_limits(color):
                mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
                    self.get_logger().info(f"{color_name} detected")
        cv2.imshow("Frame", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    color_subscriber = ColorSubscriber()
    rclpy.spin(color_subscriber)
    cv2.destroyAllWindows()
    color_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
