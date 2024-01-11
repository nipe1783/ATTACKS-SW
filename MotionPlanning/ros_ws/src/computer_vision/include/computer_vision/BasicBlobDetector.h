# pragma once
#include "computer_vision/BlobDetector.h"
#include "computer_vision/Blob.h"
#include <vector>

using namespace cv;

class BasicBlobDetector : public BlobDetector
{       
    public:
        // methods:
        BasicBlobDetector() = default;
        ~BasicBlobDetector() = default;
        std::vector<Blob> detect(Mat& frame) override;
        void calibrate(Mat& frame) override;
};