# pragma once
#include "BlobDetector.h"

using namespace cv;

class VaryingLightBlobDetector : public BlobDetector
{       
    public:

        // methods:
        VaryingLightBlobDetector() = default;
        ~VaryingLightBlobDetector() = default;
        void detect(Mat& frame, Mat& dst) override;
        void calibrate(Mat& frame) override;
        void gammaCorrection(Mat& frame, Mat& dst);
        void contrastEqualization(Mat& frame, Mat& dst);
        void Mask(Mat& frame, Mat& dst);
        static void on_alpha_trackbar(int, void*);
        static void on_intensity_thresh_trackbar(int, void*);
        std::vector<Rect> mergeNearbyContours(const std::vector<cv::Rect>& boundingBoxes, float mergeThreshold);



        // fields:
        float gamma = 0.7;
        int intensityThreshold = 47;
        double alpha = 0;
        double tau = 10;
        static const int maxValueAlphaSlider = 100;
        int alphaSlider = 0;
        int dilationSize = 9;
};