import cv2
import datetime
import os
import signal
import sys

def gstreamer_pipeline(capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=30, flip_method=0):
    return (
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){}, height=(int){}, format=(string)NV12, framerate=(fraction){}/1 ! "
        "nvvidconv flip-method={} ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink drop=true sync=false"
    ).format(capture_width, capture_height, framerate, flip_method, display_width, display_height)

def make_video_writer(width, height, fps=30, recordings_dir="../Datasets"):
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

cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Failed to open camera.")
    exit()

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
video_writer = make_video_writer(width, height)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        video_writer.write(frame)
        
finally:
    cap.release()
    video_writer.release()