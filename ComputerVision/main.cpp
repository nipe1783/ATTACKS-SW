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
    std::string videoPath = "../videos/test.mp4";
    VideoCapture cap(videoPath);
    Mat frame;
    Mat filteredFrame;
    int counter = 0;
    while(true){
        cap >> frame;
        if(frame.empty()){
            std::cout << "End of video." << std::endl;
            break;
        }
        if(counter == 0){
            blobDetector.calibrate(frame);
            counter++;
        }
        cvtColor(frame, filteredFrame, COLOR_BGR2HSV);
        GaussianBlur(filteredFrame, filteredFrame, Size(blobDetector.blurSize, blobDetector.blurSize), 0);
        inRange(filteredFrame, Scalar(blobDetector.hLow, blobDetector.sLow, blobDetector.vLow), Scalar(blobDetector.hHigh, blobDetector.sHigh, blobDetector.vHigh), filteredFrame);
        std::shared_ptr<Blob> blob = blobDetector.detect(frame);
        if(blob != nullptr){
            rectangle(frame, Point(blob->x, blob->y), Point(blob->x + blob->width, blob->y + blob->height), Scalar(0, 255, 0), 2);
        }
        Visualizer::twoFrame(frame, filteredFrame);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }

    return 0;
}