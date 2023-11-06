#include "Scripts.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <cmath>
#include "../visualizer/Visualizer.h"
#include "../blobDetection/BlobDetection.h"

using namespace cv;

void Scripts::VideoRunner(const std::string&fileName, BlobDetection& blobDetector){

    std::string videoPath = "../videos/" + fileName;
    VideoCapture cap(videoPath);
    Mat frame, dst;
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
        blobDetector.simpleDetect(frame,dst);
        imshow("Frame", frame);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }
}
