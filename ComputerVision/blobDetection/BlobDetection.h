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
    void calibrate(std::string imagePath);
    void detect(std::string imagePath);
    
    // Member variables:
    int hLow = 100;
    int hHigh = 118;
    int sLow = 28;
    int sHigh = 35;
    int vLow = 44;
    int vHigh = 90;
    int blurSize = 91;
    
    static const int maxValueH = 360/2;
    static const int maxValue = 255;
};