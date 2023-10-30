#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

// Blob Detection Imports
#include "blobDetection/BlobDetection.h"


using namespace cv;
int main()
{
    BlobDetection blobDetector;
    blobDetector.calibrate("../images/test.png");
    return 0;
}