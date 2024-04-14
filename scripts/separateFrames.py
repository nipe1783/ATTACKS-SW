import cv2 as cv
import numpy as np

def separateFrames(videoPath):
    cap = cv.VideoCapture(videoPath)
    counter = 0

    while cap.isOpened():
        ret, frame = cap.read()

        if(counter%100 == 0):
            imageName = "frame" + str(counter) + ".png"
            cv.imwrite(imageName, frame)
        
        counter = counter + 1

        if not ret:
            print("Can't receive frame. Exiting...")
            break

separateFrames("../videos/Last_flight.mp4")