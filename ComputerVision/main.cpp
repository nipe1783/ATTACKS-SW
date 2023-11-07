#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

// Blob Detection Imports
#include "blobDetector/BasicBlobDetector.h"
#include "blobDetector/VaryingLightBlobDetector.h"
#include "visualizer/Visualizer.h"
#include "benchmarking/Benchmarking.h"

using namespace cv;
int main()
{

    Mat dst;
    Mat frame = cv::imread("../images/DroneTestImages/frame_0.jpg");
    VaryingLightBlobDetector blobDetector;
    blobDetector.calibrate(frame);
    // Benchmarking::run("../images/train1", "../images/label1", blobDetector);
    for(int i = 0; i < 2000; i++){
        std::string path = "../images/DroneTestImages/frame_" + std::to_string(i) + ".jpg";
        frame = cv::imread(path);
        blobDetector.detect(frame, dst);
        Visualizer::twoFrame(frame, dst);
        waitKey(10);
    }
    return 0;
}