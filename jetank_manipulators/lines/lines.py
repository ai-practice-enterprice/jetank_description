#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetector:
    def __init__(self):
        rospy.init_node('line_detector')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
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
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
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
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    detector = LineDetector()
    rospy.spin()
    cv2.destroyAllWindows()
