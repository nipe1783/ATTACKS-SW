#pragma once
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

using namespace cv;

class Scripts
{
public:
    // Constructor and destructor:
    Scripts() = default;
    ~Scripts() = default;

    static void VideoRunner(std::string fileName);
};