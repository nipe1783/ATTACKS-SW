#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "uas_computer_vision/Blob.h"
#include "uas_computer_vision/CVImg.h"

using namespace cv;

class BlobDetector
{   

    public:
        // Public methods:
        virtual void calibrate(Mat& frame) = 0;
        virtual CVImg detect(const Mat& frame) = 0;
        virtual std::vector<Blob> detect(Mat& frame, Mat& dst) = 0; 
        static void on_low_H_thresh_trackbar(int, void*);
        static void on_high_H_thresh_trackbar(int, void*);
        static void on_low_S_thresh_trackbar(int, void*);
        static void on_high_S_thresh_trackbar(int, void*);
        static void on_low_V_thresh_trackbar(int, void*);
        static void on_high_V_thresh_trackbar(int, void*);
        static void on_area_threshold_trackbar(int, void*);
        
        // Member variables:
        int hLow_ = 0;
        int hHigh_ = 255;
        int sLow_ = 0;
        int sHigh_ = 255;
        int vLow_ = 226;
        int vHigh_ = 255;
        int blurSize_ = 3;
        int areaThreshold_ = 1;
        static const int maxValueH_ = 255;
};