#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
void showVideo()
{
    std::string video_path = "/Users/nicolasperrault/Desktop/ATTACKS-SW/OpenCVDemo/videos/video1.mp4";
    VideoCapture cap(video_path);
    Mat img;
    while(true){
        cap.read(img);
        imshow("Display window", img);
        waitKey(1);
    }
}