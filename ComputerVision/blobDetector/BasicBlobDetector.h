# pragma once
#include "BlobDetector.h"

using namespace cv;

class BasicBlobDetector : public BlobDetector
{       
    public:
        // methods:
        BasicBlobDetector() = default;
        ~BasicBlobDetector() = default;
        void detect(Mat& frame, Mat& dst) override;
        void calibrate(Mat& frame) override;
};