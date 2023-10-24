#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <torch/torch.h>

// tutorial files
#include "videoDemo/VideoDemo.cpp"
#include "imageDemo/ImageDemo.cpp"
#include "webCamDemo/WebCamDemo.cpp"
#include "torchDemo/TorchDemo.cpp"

using namespace cv;
int main()
{
    test();
    return 0;
}