# pragma once
#include "uas_computer_vision/BlobDetector.h"
#include <vector>

using namespace cv;

class BasicBlobDetector : public BlobDetector
{       
    public:
        // methods:
        BasicBlobDetector() = default;
        ~BasicBlobDetector() = default;
        CVImg detect(Mat& frame) override;
        void calibrate(Mat& frame) override;
};