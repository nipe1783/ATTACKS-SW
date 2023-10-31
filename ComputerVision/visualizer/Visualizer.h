# pragma once
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

class Visualizer
{
    private:
        /* data */
    public:
        Visualizer(/* args */);
        ~Visualizer();
        static void twoFrame(const cv::Mat& frame1,const cv::Mat& frame2);

};