#include "uas_computer_vision/BasicBlobDetector.h"
#include "uas_computer_vision/Blob.h"
#include <vector>
#include "uas_computer_vision/CVImg.h"

BasicBlobDetector::BasicBlobDetector(int hLow, int hHigh, int sLow, int sHigh, int vLow, int vHigh){
    this->hLow_ = hLow;
    this->hHigh_ = hHigh;
    this->sLow_ = sLow;
    this->sHigh_ = sHigh;
    this->vLow_ = vLow;
    this->vHigh_ = vHigh;
}

CVImg BasicBlobDetector::detect(Mat& frame){
    Mat labels, stats, centroids, dst;
    // Filtering image based on calibration values
    cvtColor(frame, dst, COLOR_BGR2HSV);
    GaussianBlur(dst, dst, Size(blurSize_, blurSize_), 0);
    inRange(dst, Scalar(hLow_, sLow_, vLow_), Scalar(hHigh_, sHigh_, vHigh_), dst);
    // Detecting blobs
    std::vector<Blob> myblobVector; // Create an empty blob vector
    int numberOfLabels = connectedComponentsWithStats(dst, labels, stats, centroids);
    int maxArea = 0;
    int secondMaxArea = 0;
    int largestBlobLabel = 0;
    int secondLargestBlobLabel = 0;
    for (int i = 1; i < numberOfLabels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if(area > maxArea) {
            maxArea = area;
            largestBlobLabel = i;
        }
    }
    if (largestBlobLabel != 0&& maxArea > areaThreshold_) { 
        int x = stats.at<int>(largestBlobLabel, cv::CC_STAT_LEFT);
        int y = stats.at<int>(largestBlobLabel, cv::CC_STAT_TOP);
        int width = stats.at<int>(largestBlobLabel, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(largestBlobLabel, cv::CC_STAT_HEIGHT);
        myblobVector.push_back(Blob(x, y, width, height, maxArea));
    }

    return CVImg(frame.cols, frame.rows, frame.cols / 2, frame.rows / 2, myblobVector);
}

std::vector<Blob> BasicBlobDetector::detect(Mat& frame, Mat& dst){
    Mat labels, stats, centroids;

    // Filtering image based on calibration values
    cvtColor(frame, dst, COLOR_BGR2HSV);
    GaussianBlur(dst, dst, Size(blurSize_, blurSize_), 0);
    inRange(dst, Scalar(hLow_, sLow_, vLow_), Scalar(hHigh_, sHigh_, vHigh_), dst);

    // Detecting blobs
    std::vector<Blob> myblobVector; // Create an empty blob vector
    int numberOfLabels = connectedComponentsWithStats(dst, labels, stats, centroids);
    int maxArea = 0;
    int secondMaxArea = 0;
    int largestBlobLabel = 0;
    int secondLargestBlobLabel = 0;
    for (int i = 1; i < numberOfLabels; i++) { // Start from 1 to skip background
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if(area > maxArea) {
            secondMaxArea = maxArea;
            maxArea = area;
            secondLargestBlobLabel = largestBlobLabel;
            largestBlobLabel = i;
        }
    }
        if (largestBlobLabel != 0&& maxArea > areaThreshold_) { 
            int x = stats.at<int>(largestBlobLabel, cv::CC_STAT_LEFT);
            int y = stats.at<int>(largestBlobLabel, cv::CC_STAT_TOP);
            int width = stats.at<int>(largestBlobLabel, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(largestBlobLabel, cv::CC_STAT_HEIGHT);
            cv::Rect bounding_box = cv::Rect(x, y, width, height);
            cv::rectangle(frame, bounding_box, cv::Scalar(255, 0, 0), 2);
            myblobVector.push_back(Blob(x, y, width, height, maxArea));
        }

        return myblobVector;
}

void BasicBlobDetector::calibrate(Mat& frame){
    std::cout << "Calibrating..." << std::endl;
    Mat frameHSV;
    Mat filteredFrame;

    namedWindow("Original Frame", WINDOW_AUTOSIZE);
    namedWindow("Filtered Frame", WINDOW_AUTOSIZE);
    
    imshow("Original Frame", frame);
    
    cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    imshow("Filtered Frame", frameHSV);

    createTrackbar("Low H", "Filtered Frame", &hLow_, maxValueH_, on_low_H_thresh_trackbar, this);
    createTrackbar("High H", "Filtered Frame", &hHigh_, maxValueH_, on_high_H_thresh_trackbar, this);
    createTrackbar("Low S", "Filtered Frame", &sLow_, maxValueH_, on_low_S_thresh_trackbar, this);
    createTrackbar("High S", "Filtered Frame", &sHigh_, maxValueH_, on_high_S_thresh_trackbar, this);
    createTrackbar("Low V", "Filtered Frame", &vLow_, maxValueH_, on_low_V_thresh_trackbar, this);
    createTrackbar("High V", "Filtered Frame", &vHigh_, maxValueH_, on_high_V_thresh_trackbar, this);
    createTrackbar("Blur Size", "Filtered Frame", &blurSize_, 100, NULL);

    while(true){

        cvtColor(frame, frameHSV, COLOR_BGR2HSV);

        // Blur the frame
        if(blurSize_ > 0 && blurSize_ % 2 == 1) {
            GaussianBlur(frameHSV, frameHSV, Size(blurSize_, blurSize_), 0);
        }
        else if(blurSize_ > 0){
            blurSize_++;
            GaussianBlur(frameHSV, frameHSV, Size(blurSize_, blurSize_), 0);
        }

        // Detect the object based on HSV Range Values
        inRange(frameHSV, Scalar(hLow_, sLow_, vLow_), Scalar(hHigh_, sHigh_, vHigh_), filteredFrame);
        
        // Show the filtered frame
        imshow("Filtered Frame", filteredFrame);

        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }
    std::cout << "Low H: " << hLow_ << std::endl;
    std::cout << "High H: " << hHigh_ << std::endl;
    std::cout << "Low S: " << sLow_ << std::endl;
    std::cout << "High S: " << sHigh_ << std::endl;
    std::cout << "Low V: " << vLow_ << std::endl;
    std::cout << "High V: " << vHigh_ << std::endl;
    std::cout << "Blur Size: " << blurSize_ << std::endl;
    std::cout << "Finished Calibrating." << std::endl;
}