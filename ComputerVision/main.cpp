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
    std::string videoPath = "../videos/test.mp4";
    VideoCapture cap(videoPath);
    Mat frame;
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
        frame = blobDetector.detect(frame);
        imshow("Frame", frame);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }
    
    // blobDetector.detect(frame);
    return 0;
}