#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
void showWebCam()
{
    VideoCapture cap(0);
    Mat img;
    while(true){
        cap.read(img);
        imshow("Display window", img);
        waitKey(1);
    }
}