import cv2 as cv

max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
blur = 0
window_capture_name = "Original Image"
window_detection_name = "HSV Image"
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)

def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)

def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)

def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)

def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)

def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)

def blur_trackbar(self):
    pass
    
def calibrateHSV(imgPath):
    cv.namedWindow(window_capture_name)
    cv.namedWindow(window_detection_name)

    cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
    cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
    cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
    cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
    cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
    cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
    cv.createTrackbar("Blur Size", window_detection_name, blur, 100, blur_trackbar)

    while True:
        img = cv.imread(imgPath)

        if img is None:
            print("Could not read the image.")
            break

        cv.imshow(window_capture_name, img)
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        blurSize = cv.getTrackbarPos("Blur Size", window_detection_name)
        if(blurSize > 0 and blurSize % 2 == 1):
            hsv = cv.GaussianBlur(hsv, (blurSize, blurSize), 0)
        elif(blurSize > 0):
            blurSize = blurSize + 1
            hsv = cv.GaussianBlur(hsv, (blurSize, blurSize), 0)

        frame_threshold = cv.inRange(hsv, (low_H, low_S, low_V), (high_H, high_S, high_V))

        cv.imshow(window_detection_name, frame_threshold)

        key = cv.waitKey(30)
        if key == ord('q') or key == 27:
            break
    
    print("Low H:", low_H)
    print("High H:", high_H)
    print("Low S:", low_S)
    print("High S:", high_S)
    print("Low V:", low_V)
    print("High V:", high_V)
    print("Blur Size:", blurSize)

calibrateHSV("frame900.png")