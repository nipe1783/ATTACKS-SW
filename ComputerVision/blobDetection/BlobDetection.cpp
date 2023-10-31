#include "BlobDetection.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

void BlobDetection::calibrate(Mat frame)
{
    std::cout << "Calibrating..." << std::endl;
    Mat frameHSV;
    Mat filteredFrame;

    namedWindow("Original Frame", WINDOW_AUTOSIZE);
    namedWindow("Filtered Frame", WINDOW_AUTOSIZE);
    
    imshow("Original Frame", frame);
    
    cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    createTrackbar("Low H", "Filtered Frame", &hLow, maxValueH, on_low_H_thresh_trackbar, this);
    createTrackbar("High H", "Filtered Frame", &hHigh, maxValueH, on_high_H_thresh_trackbar, this);
    createTrackbar("Low S", "Filtered Frame", &sLow, maxValueH, on_low_S_thresh_trackbar, this);
    createTrackbar("High S", "Filtered Frame", &sHigh, maxValueH, on_high_S_thresh_trackbar, this);
    createTrackbar("Low V", "Filtered Frame", &vLow, maxValueH, on_low_V_thresh_trackbar, this);
    createTrackbar("High V", "Filtered Frame", &vHigh, maxValueH, on_high_V_thresh_trackbar, this);
    createTrackbar("Blur Size", "Filtered Frame", &blurSize, 100, NULL);

    while(true){

        cvtColor(frame, frameHSV, COLOR_BGR2HSV);

        // Blur the frame
        if(blurSize > 0 && blurSize % 2 == 1) {
            GaussianBlur(frameHSV, frameHSV, Size(blurSize, blurSize), 0);
        }
        else if(blurSize > 0){
            blurSize++;
            GaussianBlur(frameHSV, frameHSV, Size(blurSize, blurSize), 0);
        }

        // Detect the object based on HSV Range Values
        inRange(frameHSV, Scalar(hLow, sLow, vLow), Scalar(hHigh, sHigh, vHigh), filteredFrame);
        
        // Show the filtered frame
        imshow("Filtered Frame", filteredFrame);

        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }
    std::cout << "Low H: " << hLow << std::endl;
    std::cout << "High H: " << hHigh << std::endl;
    std::cout << "Low S: " << sLow << std::endl;
    std::cout << "High S: " << sHigh << std::endl;
    std::cout << "Low V: " << vLow << std::endl;
    std::cout << "High V: " << vHigh << std::endl;
    std::cout << "Blur Size: " << blurSize << std::endl;
    std::cout << "Finished Calibrating." << std::endl;
}


Mat& BlobDetection::detect(Mat& frame)
{
    Mat frameHSV, filteredFrame, labels, stats, centroids;

    // Filtering image based on calibration values
    cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    GaussianBlur(frameHSV, frameHSV, Size(blurSize, blurSize), 0);
    inRange(frameHSV, Scalar(hLow, sLow, vLow), Scalar(hHigh, sHigh, vHigh), filteredFrame);

    // Detecting blobs
    int numberOfLabels = connectedComponentsWithStats(filteredFrame, labels, stats, centroids);
    int largestBlobLabel = 0;
    int maxArea = 0;
    for (int i = 1; i < numberOfLabels; i++) { // Start from 1 to skip background
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if(area > maxArea) {
            maxArea = area;
            largestBlobLabel = i;
        }
    }

    // bounding largest blob
    if (largestBlobLabel != 0) { 
        int x = stats.at<int>(largestBlobLabel, cv::CC_STAT_LEFT);
        int y = stats.at<int>(largestBlobLabel, cv::CC_STAT_TOP);
        int width = stats.at<int>(largestBlobLabel, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(largestBlobLabel, cv::CC_STAT_HEIGHT);
        rectangle(frame, Point(x, y), Point(x + width, y + height), Scalar(0, 255, 0), 2);
    }

    // imshow("Original with Largest Blob Bounded", frame);
    // waitKey(0);
    return frame;
}

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

void BlobDetection::on_low_S_thresh_trackbar(int pos, void* userdata)
{
    BlobDetection* instance = (BlobDetection*)userdata;
    instance->sLow = min(instance->sHigh-1, instance->sLow);
    setTrackbarPos("Low S", "Filtered Frame", instance->sLow);
}

void BlobDetection::on_high_S_thresh_trackbar(int pos, void* userdata)
{
    BlobDetection* instance = (BlobDetection*)userdata;
    instance->sHigh = max(instance->sHigh, instance->sLow+1);
    setTrackbarPos("High S", "Filtered Frame", instance->sHigh);
}

void BlobDetection::on_low_V_thresh_trackbar(int pos, void* userdata)
{
    BlobDetection* instance = (BlobDetection*)userdata;
    instance->vLow = min(instance->vHigh-1, instance->vLow);
    setTrackbarPos("Low V", "Filtered Frame", instance->vLow);
}

void BlobDetection::on_high_V_thresh_trackbar(int pos, void* userdata)
{
    BlobDetection* instance = (BlobDetection*)userdata;
    instance->vHigh = max(instance->vHigh, instance->vLow+1);
    setTrackbarPos("High V", "Filtered Frame", instance->vHigh);
}