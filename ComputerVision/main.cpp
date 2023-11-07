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
#include "scripts/Scripts.h"


using namespace cv;
int main()
{
    Scripts script;
    VaryingLightBlobDetector VLBlobDetector;
    BasicBlobDetector basicBlobDetector;

    
    Scripts::videoRunner("DroneTestFootage.mp4", VLBlobDetector);
    return 0;
}