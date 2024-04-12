import cv2
import datetime
import os

def gstreamer_pipeline(device='/dev/video1', capture_width=1280, capture_height=720, framerate=30):
    return (
        f"v4l2src device={device} ! "
        f"image/jpeg, width=(int){capture_width}, height=(int){capture_height}, "
        f"framerate=(fraction){framerate}/1 ! "
        "jpegdec ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink drop=true sync=false"
    )

def main():
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture frame.")
                break
            cv2.imshow("USB Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
