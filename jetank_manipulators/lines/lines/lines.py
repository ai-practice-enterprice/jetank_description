#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.bridge = CvBridge()
        
        # ROS2 subscriber (note: topic name may need adjustment)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10  # Queue size
        )
        
        # YELLOW HSV RANGE
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        
        # GREEN HSV RANGE
        self.lower_green = np.array([40, 70, 50])
        self.upper_green = np.array([80, 255, 255])

    def detect_lines(self, mask):
        edges = cv2.Canny(mask, 50, 150)
        return cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50,
                             minLineLength=50, maxLineGap=10)

    def draw_lines(self, image, lines, color):
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), color, 2)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create masks
            yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
            
            # Detect lines
            yellow_lines = self.detect_lines(yellow_mask)
            green_lines = self.detect_lines(green_mask)
            
            # Draw lines
            output = cv_image.copy()
            self.draw_lines(output, yellow_lines, (0, 255, 255))  # Yellow (BGR)
            self.draw_lines(output, green_lines, (0, 255, 0))       # Green
            
            # Display
            cv2.imshow('Detected Lines', output)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    detector = LineDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    
    detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()