#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

// Blob Detection Imports
#include "BlobDetection/BlobDetection.h"


using namespace cv;
int main()
{
    BlobDetection blobDetector;
    blobDetector.calibrate("test");
    return 0;
}