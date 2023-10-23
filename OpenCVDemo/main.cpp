#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

// tutorial files
#include "videoDemo/VideoDemo.cpp"
#include "imageDemo/ImageDemo.cpp"
#include "webCamDemo/WebCamDemo.cpp"
#include "yoloDemo/YoloDemo.cpp"

using namespace cv;
int main()
{
    detectImage("/Users/nicolasperrault/Desktop/ATTACKS-SW/OpenCVDemo/images/demo2.png");
    return 0;
}