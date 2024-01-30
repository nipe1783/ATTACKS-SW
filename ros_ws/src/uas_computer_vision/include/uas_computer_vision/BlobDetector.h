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
        virtual CVImg detect(Mat& frame) = 0;
        virtual std::vector<Blob> detect(Mat& frame, Mat& dst) = 0; 
        static void on_low_H_thresh_trackbar(int, void*);
        static void on_high_H_thresh_trackbar(int, void*);
        static void on_low_S_thresh_trackbar(int, void*);
        static void on_high_S_thresh_trackbar(int, void*);
        static void on_low_V_thresh_trackbar(int, void*);
        static void on_high_V_thresh_trackbar(int, void*);
        static void on_area_threshold_trackbar(int, void*);
        
        // Member variables:
        int hLow = 0;
        int hHigh = 255;
        int sLow = 0;
        int sHigh = 255;
        int vLow = 226;
        int vHigh = 255;
        int blurSize = 3;
        int areaThreshold = 1;
        static const int maxValueH = 255;
};