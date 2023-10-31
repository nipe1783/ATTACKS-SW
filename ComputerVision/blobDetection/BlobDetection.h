#pragma once
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

class BlobDetection
{
private:
    // Static callback methods for trackbars:
    static void on_low_H_thresh_trackbar(int, void*);
    static void on_high_H_thresh_trackbar(int, void*);
    static void on_low_S_thresh_trackbar(int, void*);
    static void on_high_S_thresh_trackbar(int, void*);
    static void on_low_V_thresh_trackbar(int, void*);
    static void on_high_V_thresh_trackbar(int, void*);

public:
    // Constructor and destructor:
    BlobDetection() = default;
    ~BlobDetection() = default;

    // Public methods:
    void calibrate(Mat frame);
    Mat& detect(Mat& frame);
    
    // Member variables:
    int hLow = 150;
    int hHigh = 180;
    int sLow = 69;
    int sHigh = 143;
    int vLow = 11;
    int vHigh = 180;
    int blurSize = 91;
    
    static const int maxValueH = 360/2;
    static const int maxValue = 255;
};