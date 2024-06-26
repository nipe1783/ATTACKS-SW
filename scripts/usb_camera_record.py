import cv2
import datetime
import os
import signal
import sys

def make_video_writer(width, height, fps=30, recordings_dir="../Recordings"):
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
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Failed to open camera.")
    sys.exit(1)

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS) or 30

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
    video_writer.release()