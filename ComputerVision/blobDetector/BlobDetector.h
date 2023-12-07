#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../blob/Blob.h"

using namespace cv;

class BlobDetector
{   

    public:
        // Public methods:
        virtual void calibrate(Mat& frame) = 0;
        virtual std::vector<Blob> detect(Mat& frame, Mat& dst) = 0;
        static void on_low_H_thresh_trackbar(int, void*);
        static void on_high_H_thresh_trackbar(int, void*);
        static void on_low_S_thresh_trackbar(int, void*);
        static void on_high_S_thresh_trackbar(int, void*);
        static void on_low_V_thresh_trackbar(int, void*);
        static void on_high_V_thresh_trackbar(int, void*);
        static void on_area_threshold_trackbar(int, void*);
        
        // Member variables:
        int hLow = 92;
        int hHigh = 125;
        int sLow = 14;
        int sHigh = 90;
        int vLow = 53;
        int vHigh = 222;
        int blurSize = 13;
        int areaThreshold = 700;
        static const int maxValueH = 255;
};