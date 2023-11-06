#pragma once
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../blob/Blob.h"
#include <memory>

using namespace cv;

class BlobDetector
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
    BlobDetector() = default;
    ~BlobDetector() = default;

    // Public methods:
    virtual void calibrate(Mat& frame) = 0;
    virtual void detect(Mat& frame, Mat& dst) = 0;
    
    // Member variables:
    int hLow = 61;
    int hHigh = 141;
    int sLow = 29;
    int sHigh = 131;
    int vLow = 82;
    int vHigh = 203;
    int blurSize = 15;
    static const int maxValueH = 255;
    static const int maxValue = 255;
};