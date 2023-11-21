# pragma once
#include "BlobDetector.h"
#include "../blob/Blob.h"
#include <vector>

using namespace cv;

class DoGBlobDetector : public BlobDetector
{       
    public:
        // methods:
        DoGBlobDetector() = default;
        ~DoGBlobDetector() = default;
        std::vector<Blob> detect(Mat& frame, Mat& dst) override;
        void calibrate(Mat& frame) override;
};