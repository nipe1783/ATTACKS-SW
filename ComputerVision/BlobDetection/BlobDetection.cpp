#include "BlobDetection.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

void BlobDetection::calibrate(std::string imagePath)
{
    std::cout << "Calibrating..." << std::endl;

    Mat image = imread(imagePath, IMREAD_COLOR);
    Mat filteredImage;

    namedWindow("Original Frame", WINDOW_AUTOSIZE);
    namedWindow("Filtered Frame", WINDOW_AUTOSIZE);
    
    imshow("Original Frame", image);
    
    cvtColor(image, filteredImage, COLOR_BGR2HSV);
    createTrackbar("Low H", "Filtered Frame", &hLow, maxValueH, on_low_H_thresh_trackbar, this);
    createTrackbar("High H", "Filtered Frame", &hHigh, maxValueH, on_high_H_thresh_trackbar, this);

    imshow("Filtered Frame", filteredImage);
    
    waitKey(0);
    
    std::cout << "Finished Calibrating." << std::endl;
}

void BlobDetection::detect(std::string imagePath)
{
    std::cout << "Detecting..." << std::endl;
}

// Static callback methods
void BlobDetection::on_low_H_thresh_trackbar(int pos, void* userdata)
{
    BlobDetection* instance = (BlobDetection*)userdata;
    instance->hLow = min(instance->hHigh-1, instance->hLow);
    setTrackbarPos("Low H", "Filtered Frame", instance->hLow);
}

void BlobDetection::on_high_H_thresh_trackbar(int pos, void* userdata)
{
    BlobDetection* instance = (BlobDetection*)userdata;
    instance->hHigh = max(instance->hHigh, instance->hLow+1);
    setTrackbarPos("High H", "Filtered Frame", instance->hHigh);
}
