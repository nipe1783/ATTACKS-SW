#pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

class BlobDetector
{   

    public:
        // Public methods:
        virtual void calibrate(Mat& frame) = 0;
        virtual void detect(Mat& frame, Mat& dst) = 0;
        static void on_low_H_thresh_trackbar(int, void*);
        static void on_high_H_thresh_trackbar(int, void*);
        static void on_low_S_thresh_trackbar(int, void*);
        static void on_high_S_thresh_trackbar(int, void*);
        static void on_low_V_thresh_trackbar(int, void*);
        static void on_high_V_thresh_trackbar(int, void*);
        
        // Member variables:
        int hLow = 0;
        int hHigh = 255;
        int sLow = 0;
        int sHigh = 255;
        int vLow = 0;
        int vHigh = 255;
        int blurSize = 1;
        double areaThreshold = 0.0;
        static const int maxValueH = 255;
};