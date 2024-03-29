#pragma once
#include "uas_computer_vision/BlobDetector.h"
#include <vector>

using namespace cv;

class BasicBlobDetector : public BlobDetector
{       
    public:
        // methods:
        BasicBlobDetector(int hLow, int hHigh, int sLow, int sHigh, int vLow, int vHigh);
        BasicBlobDetector() = default;
        ~BasicBlobDetector() = default;
        CVImg detect(const Mat& frame) override;
        std::vector<Blob> detect(Mat& frame, Mat& dst) override;
        void calibrate(Mat& frame) override;
};