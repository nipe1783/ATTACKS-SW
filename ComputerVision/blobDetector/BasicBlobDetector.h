# pragma once
#include "BlobDetector.h"

using namespace cv;

class BasicBlobDetector : public BlobDetector
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

        // methods:
        BasicBlobDetector() = default;
        ~BasicBlobDetector() = default;
        void detect(Mat& frame, Mat& dst) override;
        void calibrate(Mat& frame) override;

        // fields:
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