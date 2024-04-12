import cv2
import datetime
import os
import signal
import sys

def gstreamer_pipeline(capture_width=1280, capture_height=720, framerate=30):
    return (
        f"v4l2src device=/dev/video1 ! "
        f"video/x-raw, width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)YUY2, framerate=(fraction){framerate}/1 ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=true sync=false"
    )

def make_video_writer(width, height, fps=30, recordings_dir="Recordings"):
    recordings_path = os.path.join(os.getcwd(), recordings_dir)
    if not os.path.exists(recordings_path):
        os.makedirs(recordings_path)

    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = os.path.join(recordings_path, f"Recording_{current_time}.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(filename, fourcc, fps, (width, height))
    return out

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Saving the video and exiting.')
    cap.release()
    video_writer.release()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Initialize video capture with GStreamer pipeline
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Failed to open camera.")
    sys.exit(1)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS) or 30  # Use default fps if detection fails

video_writer = make_video_writer(width, height, fps)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        video_writer.write(frame)

finally:
    cap.release()