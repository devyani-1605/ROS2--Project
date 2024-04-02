#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import serial 
import json 

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.bridge = CvBridge()

    def run(self):
        cap = cv2.VideoCapture(0)
        arduino = serial.Serial('/dev/ttyUSB0', 115200)

        while cap.isOpened():
            ret, frame = cap.read()
            frame = cv2.resize(frame, (640, 480))

            if not ret:
                self.get_logger().error("Error reading frame from webcam")
                break

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            lower_blue = np.array([100, 50, 50])
            upper_blue = np.array([140, 255, 255])

            mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                max_contour = max(contours, key=cv2.contourArea)
            else:
                max_contour = None

            if max_contour is not None:
                contour_area = cv2.contourArea(max_contour)
                if contour_area > 400:
                    cv2.drawContours(frame, [max_contour], -1, (0, 255, 0), 2)
                    M = cv2.moments(max_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        # cx=320-cx
                        # cy=240-cy
                        self.get_logger().info("Centroid Coordinate: ({}, {})".format((int)((320 - cx) / 20), (int)((240 - cy) / 20)))
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), 3)
                        try:
                            json_data = {"x": (int)((320 - cx) / 20), "y": (int)((240 - cy) / 20)}  # key : value
                            json_send = json.dumps(json_data)
                            arduino.write(json_send.encode('utf-8'))
                        except OSError:
                            arduino = serial.Serial('/dev/ttyUSB0', 115200)
                            self.get_logger().error("error occurred")

            # Publish the frame as ROS2 image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_image)

        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    webcam_publisher.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

