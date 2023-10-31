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
    void updateFilteredImage(const Mat& src, Mat& dest);
    
    // Member variables:
    int hLow = 0;
    int hHigh = 0;
    int sLow = 0;
    int sHigh = 0;
    int vLow = 0;
    int vHigh = 0;
    
    static const int maxValueH = 360/2;
    static const int maxValue = 255;
};