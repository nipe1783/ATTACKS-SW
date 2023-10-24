#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

// tutorial files
#include "videoDemo/VideoDemo.cpp"
#include "imageDemo/ImageDemo.cpp"
#include "webCamDemo/WebCamDemo.cpp"

using namespace cv;
int main()
{
    showWebCam();
    return 0;
}