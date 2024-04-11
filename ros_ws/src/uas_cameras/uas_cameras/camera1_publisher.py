import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os
import datetime

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer_period = 1/30
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera.')
            rclpy.shutdown()

    def gstreamer_pipeline(self, capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=30, flip_method=0):
        return (
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){}, height=(int){}, format=(string)NV12, framerate=(fraction){}/1 ! "
            "nvvidconv flip-method={} ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true sync=false"
        ).format(capture_width, capture_height, framerate, flip_method, display_width, display_height)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame.')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing camera frame')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()