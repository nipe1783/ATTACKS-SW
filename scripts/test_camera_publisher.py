#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import datetime
import os
import sys
import signal

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 5)
        self.timer_period = 1 / 30  # Timer frequency matches the camera FPS
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        self.video_writer = self.make_video_writer()

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera with GStreamer pipeline.')
            rclpy.shutdown()

    def gstreamer_pipeline(self, capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=30, flip_method=0):
        return (
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){}, height=(int){}, format=(string)NV12, framerate=(fraction){}/1 ! "
            "nvvidconv flip-method={} ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true sync=false"
        ).format(capture_width, capture_height, framerate, flip_method, display_width, display_height)

    def make_video_writer(self, recordings_dir="../Recordings"):
        recordings_path = os.path.join(os.getcwd(), recordings_dir)
        if not os.path.exists(recordings_path):
            os.makedirs(recordings_path)

        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = 30  # Adjust as needed
        current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = os.path.join(recordings_path, f"Recording_{current_time}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
        return cv2.VideoWriter(filename, fourcc, fps, (width, height))

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame.')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)
        self.video_writer.write(frame)
        self.get_logger().info('Publishing camera frame')

    def signal_handler(self, sig, frame):
        self.get_logger().info('You pressed Ctrl+C! Saving the video and exiting.')
        self.shutdown()

    def shutdown(self):
        self.cap.release()
        self.video_writer.release()
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    signal.signal(signal.SIGINT, camera_publisher.signal_handler)
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.shutdown()

if __name__ == '__main__':
    main()
