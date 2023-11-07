#pragma once
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include "../blobDetector/BlobDetector.h"

using namespace cv;

class Scripts
{
public:
    // Constructor and destructor:
    Scripts() = default;
    ~Scripts() = default;

    static void cameraRunner(const int cameraNumber, BlobDetector& blobDetector);
    static void videoRunner(const std::string&fileName, BlobDetector& blobDetector);
    static void imageRunner(const std::string&fileName, BlobDetector& blobDetector);
};