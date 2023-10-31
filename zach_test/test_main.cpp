#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"

#include <iostream>
using namespace cv;
const String rgb_detection_name = "RGB Threshold";

int low_R = 0, low_G = 80, low_B = 0;
int high_R = 255, high_G = 255, high_B = 255;
void on_low_R_thresh_trackbar(int, void *) {
    // Handle the low R trackbar
    if (low_R >= high_R) {
        low_R = high_R - 1;
        setTrackbarPos("Low R", rgb_detection_name, low_R);
    }
}

void on_high_R_thresh_trackbar(int, void *) {
    // Handle the high R trackbar
    if(high_R == 0){
        setTrackbarPos("High R", rgb_detection_name, 255); 
    }
    if (high_R <= low_R) {
        high_R = low_R + 1;
        setTrackbarPos("High R", rgb_detection_name, high_R);
    }
}

void on_low_G_thresh_trackbar(int, void *) {
    // Handle the low G trackbar
    if (low_G >= high_G) {
        low_G = high_G - 1;
        setTrackbarPos("Low G", rgb_detection_name, low_G);
    }
}

void on_high_G_thresh_trackbar(int, void *) {
    // Handle the high G trackbar
    if(high_G == 0){
        setTrackbarPos("High G", rgb_detection_name, 255); 
    }
    if (high_G <= low_G) {
        high_G = low_G + 1;
        setTrackbarPos("High G", rgb_detection_name, high_G);
    }
}

void on_low_B_thresh_trackbar(int, void *) {
    // Handle the low B trackbar
    if (low_B >= high_B) {
        low_B = high_B - 1;
        setTrackbarPos("Low B", rgb_detection_name, low_B);
    }
}

void on_high_B_thresh_trackbar(int, void *) {
    // Handle the high B trackbar
    if(high_B == 0){
        setTrackbarPos("High B", rgb_detection_name, 255); 
    }
    if (high_B <= low_B) {
        high_B = low_B + 1;
        setTrackbarPos("High B", rgb_detection_name, high_B);
    }
}

const int max_value_H = 179;
const int max_value = 255;
const String window_detection_name = "HSV Threshold";

int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

void on_low_H_thresh_trackbar(int, void *) {
    // Handle the low H trackbar
    if (low_H >= high_H) {
        low_H = high_H - 1;
        setTrackbarPos("Low H", window_detection_name, low_H);
    }
}

void on_high_H_thresh_trackbar(int, void *) {
    // Handle the high H trackbar
    if(high_H == 0){
        setTrackbarPos("High H", window_detection_name, 179); 
    }
    if (high_H <= low_H) {
        high_H = low_H + 1;
        setTrackbarPos("High H", window_detection_name, high_H);
    }
}

void on_low_S_thresh_trackbar(int, void *) {
    // Handle the low S trackbar
    if (low_S >= high_S) {
        low_S = high_S - 1;
        setTrackbarPos("Low S", window_detection_name, low_S);
    }
}

void on_high_S_thresh_trackbar(int, void *) {
    // Handle the high S trackbar
    if(high_S == 0){
        setTrackbarPos("High S", window_detection_name, 255); 
    }
    if (high_S <= low_S) {
        high_S = low_S + 1;
        setTrackbarPos("High S", window_detection_name, high_S);
    }
}

void on_low_V_thresh_trackbar(int, void *) {
    // Handle the low V trackbar
    if (low_V >= high_V) {
        low_V = high_V - 1;
        setTrackbarPos("Low V", window_detection_name, low_V);
    }
}

void on_high_V_thresh_trackbar(int, void *) {
    // Handle the high V trackbar
     if(high_V == 0){
        setTrackbarPos("High V", window_detection_name, 255); 
    }
    if (high_V <= low_V) {
        high_V = low_V + 1;
        setTrackbarPos("High V", window_detection_name, high_V);
    }
}

int main(int argc, char *argv[])
{
    VideoCapture cap("/Users/zachchen/Desktop/ATTACKS-SW/zach_test/testvid.mp4"); //test video
    //VideoCapture cap(0); webcam
    namedWindow(rgb_detection_name);
    namedWindow(window_detection_name);

    // Trackbars to set thresholds for HSV values
    // createTrackbar("Low H", window_detection_name, nullptr, max_value_H, on_low_H_thresh_trackbar);
    // createTrackbar("High H", window_detection_name, nullptr, max_value_H, on_high_H_thresh_trackbar);
    // createTrackbar("Low S", window_detection_name, nullptr, max_value, on_low_S_thresh_trackbar);
    // createTrackbar("High S", window_detection_name, nullptr, max_value, on_high_S_thresh_trackbar);
    // createTrackbar("Low V", window_detection_name, nullptr, max_value, on_low_V_thresh_trackbar);
    // createTrackbar("High V", window_detection_name, nullptr, max_value, on_high_V_thresh_trackbar);

    // createTrackbar("Low R", rgb_detection_name, nullptr, 255, on_low_R_thresh_trackbar);
    // createTrackbar("High R", rgb_detection_name, nullptr, 255, on_high_R_thresh_trackbar);
    // createTrackbar("Low G", rgb_detection_name, nullptr, 255, on_low_G_thresh_trackbar);
    // createTrackbar("High G", rgb_detection_name, nullptr, 255, on_high_G_thresh_trackbar);
    // createTrackbar("Low B", rgb_detection_name, nullptr, 255, on_low_B_thresh_trackbar);
    // createTrackbar("High B", rgb_detection_name, nullptr, 255, on_high_B_thresh_trackbar);

    Mat frame, frame_HSV, frame_gray, frame_threshold, im_with_keypoints, rgb_threshold;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;

    // Change thresholds
    params.filterByColor = false;
    params.blobColor = 0;

   params.minThreshold  = 125;
    params.maxThreshold  = 255;
    params.thresholdStep = 10;

    // params.thresholdStep = 10;
    params.minRepeatability = 15;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 500;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.7;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.1;

    // Filter by Inertia - how elongated a shape is. 1 for a circle
    params.filterByInertia = false;
    // params.minInertiaRatio = 0.2;

    // Storage for blobs
    std::vector<KeyPoint> keypoints;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
        {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

        // filter based on HSV Range Values
        // inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        inRange(frame_HSV, Scalar(0, 0, 50), Scalar(179, 255, 255), frame_threshold);


        // filter based on RGB Range Values

        inRange(frame, Scalar(low_B, low_G, low_R), Scalar(high_B, high_G, high_R), rgb_threshold);

        // Show the frames
        imshow("RGB", frame);
        // imshow("HSV", frame_HSV);
        // imshow(window_detection_name, frame_threshold);
        // imshow(rgb_detection_name, rgb_threshold);
        // imshow("Gray", frame_gray);

        // Detect blobs
        detector->detect(frame_threshold, keypoints);
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        drawKeypoints(frame_threshold, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        for (const KeyPoint &keypoint : keypoints)
        {
            Point2f point = keypoint.pt;
            float size = keypoint.size;

            // Calculate the coordinates for the rectangle
            Point2f top_left(point.x - size / 2, point.y - size / 2);
            Point2f bottom_right(point.x + size / 2, point.y + size / 2);

            // Draw the rectangle
            rectangle(im_with_keypoints, top_left, bottom_right, Scalar(0, 255, 0), 2);
        }

        imshow("keypoints on gray", im_with_keypoints);

        char key = (char)waitKey(1); //30 fps
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}
