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
    Benchmarking benchmark;

    benchmark.runBasic("../images", "../binaryImages/", basicBlobDetector);
    // benchmark.runVarying("../images", "../binaryImages/", VLBlobDetector);

    // Scripts::videoRunner("DJI_20231117032318_0005_S.MP4", basicBlobDetector);
    // Scripts::cameraRunner(0, basicBlobDetector);

    // Saving every 50 frames as an image
    // std::string videoPath = "/home/alex/Documents/ATTACKS-SW/ComputerVision/videos/DroneTestFootage.mp4";
    // VideoCapture cap(videoPath);
    // Mat frame;
    // std::string name;
    // int counter = 0;
    // while(true){
    //     cap >> frame;
    //     if(counter%50 == 0){
    //         name = "frame_" + std::to_string(counter) + ".png";
    //         imwrite("/home/alex/Documents/ATTACKS-SW/ComputerVision/images/" + name, frame);
    //     }
    //     counter++;
    //     if(frame.empty()){
    //         break;
    //     }
    // }

    return 0;
}