# pragma once
#include "BlobDetector.h"

using namespace cv;

class DoGBlobDetector : public BlobDetector
{       
    public:
        // methods:
        DoGBlobDetector() = default;
        ~DoGBlobDetector() = default;
        void detect(Mat& frame, Mat& dst) override;
        void calibrate(Mat& frame) override;
};