#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
void showImage()
{
    std::string image_path = "/Users/nicolasperrault/Desktop/ATTACKS-SW/OpenCVDemo/images/demo2.png";
    Mat img = imread(image_path, IMREAD_COLOR);
    Mat imgGray;
    Mat imgBlur;
    Mat imgCanny;
    cvtColor(img, imgGray, COLOR_BGR2GRAY);
    GaussianBlur(img, imgBlur, Size(7,7), 0);
    Canny(imgBlur, imgCanny, 50, 100);
    dilate(imgCanny, imgCanny, getStructuringElement(MORPH_RECT, Size(3,3)));
    imshow("Display window", imgCanny);
    int k = waitKey(0); // Wait for a keystroke in the window
}