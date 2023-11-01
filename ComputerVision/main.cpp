#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

// Blob Detection Imports
#include "blobDetection/BlobDetection.h"

// Visualizer Imports
#include "visualizer/Visualizer.h"

using namespace cv;
int main()
{
    BlobDetection blobDetector;
    std::string imagePath = "../images/EORSSD/test-images/0004.jpg"; 
    Mat frame = imread(imagePath);
    Mat filteredFrame;

    imshow("Original", frame);
    waitKey(0);
    return 0;
}