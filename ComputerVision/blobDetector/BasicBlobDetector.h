# pragma once
#include "BlobDetector.h"
#include "../blob/Blob.h"
#include <vector>

using namespace cv;

class BasicBlobDetector : public BlobDetector
{       
    public:
        // methods:
        BasicBlobDetector() = default;
        ~BasicBlobDetector() = default;
        std::vector<Blob> detect(Mat& frame, Mat& dst) override;
        void calibrate(Mat& frame) override;
};