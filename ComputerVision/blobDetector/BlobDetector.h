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
        void DoGFilter(Mat& frame, Mat& dst);
        static void on_low_H_thresh_trackbar(int, void*);
        static void on_high_H_thresh_trackbar(int, void*);
        static void on_low_S_thresh_trackbar(int, void*);
        static void on_high_S_thresh_trackbar(int, void*);
        static void on_low_V_thresh_trackbar(int, void*);
        static void on_high_V_thresh_trackbar(int, void*);
        static void on_area_threshold_trackbar(int, void*);
        static void on_sigma_1_thresh_trackbar(int, void*);
        static void on_sigma_2_thresh_trackbar(int, void*);
        
        // Member variables:
        int hLow = 92;
        int hHigh = 125;
        int sLow = 27;
        int sHigh = 117;
        int vLow = 28;
        int vHigh = 227;
        int blurSize = 13;
        int sigma1 = 2;
        int sigma2 = 5;
        int areaThreshold = 700;
        static const int maxValueH = 255;
};