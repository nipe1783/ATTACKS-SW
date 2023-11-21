#include "../DoGBlobDetector.h"
#include "../../blob/Blob.h"
#include <vector>

std::vector<Blob> DoGBlobDetector::detect(Mat& frame, Mat& dst){
    Mat labels, stats, centroids;

    // Filtering image based on calibration values
    cvtColor(frame, dst, COLOR_BGR2HSV);
    DoGFilter(dst, dst);
    inRange(dst, Scalar(hLow, sLow, vLow), Scalar(hHigh, sHigh, vHigh), dst);

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
    if (largestBlobLabel != 0&& maxArea > areaThreshold) { 
        int x = stats.at<int>(largestBlobLabel, cv::CC_STAT_LEFT);
        int y = stats.at<int>(largestBlobLabel, cv::CC_STAT_TOP);
        int width = stats.at<int>(largestBlobLabel, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(largestBlobLabel, cv::CC_STAT_HEIGHT);
        cv::Rect bounding_box = cv::Rect(x, y, width, height);
        cv::rectangle(frame, bounding_box, cv::Scalar(255, 0, 0), 2);
        myblobVector.push_back(Blob(x, y, width, height, maxArea));
    }

    if (secondLargestBlobLabel != 0 && secondMaxArea > areaThreshold) { 
        int x = stats.at<int>(secondLargestBlobLabel, cv::CC_STAT_LEFT);
        int y = stats.at<int>(secondLargestBlobLabel, cv::CC_STAT_TOP);
        int width = stats.at<int>(secondLargestBlobLabel, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(secondLargestBlobLabel, cv::CC_STAT_HEIGHT);
        cv::Rect bounding_box = cv::Rect(x, y, width, height);
        cv::rectangle(frame, bounding_box, cv::Scalar(0, 0, 255), 2);
        myblobVector.push_back(Blob(x, y, width, height, secondMaxArea));
    }
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dst.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    int maxAreaContourIndex = -1;
    maxArea = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if(area > maxArea){
            maxArea = area;
            maxAreaContourIndex = i;
        }
    }
    if(maxAreaContourIndex != -1) {
        Mat mask = Mat::zeros(dst.size(), dst.type());
        cv::drawContours(mask, contours, maxAreaContourIndex, Scalar(255), cv::FILLED);
        cv::Rect boundingBox = cv::boundingRect(contours[maxAreaContourIndex]);
        cv::rectangle(frame, boundingBox, cv::Scalar(255, 0, 0), 2);
        dst = dst & mask;
    }
    return myblobVector;
}

void DoGBlobDetector::calibrate(Mat& frame){
    std::cout << "Calibrating..." << std::endl;
    Mat frameHSV;
    Mat filteredFrame;

    namedWindow("Original Frame", WINDOW_AUTOSIZE);
    // namedWindow("Filtered Frame", WINDOW_AUTOSIZE);
    namedWindow("Filtered Frame", WINDOW_NORMAL);
    resizeWindow("Filtered Frame", 1280, 720);
    
    imshow("Original Frame", frame);
    
    cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    imshow("Filtered Frame", frameHSV);

    createTrackbar("Low H", "Filtered Frame", &hLow, maxValueH, on_low_H_thresh_trackbar, this);
    createTrackbar("High H", "Filtered Frame", &hHigh, maxValueH, on_high_H_thresh_trackbar, this);
    createTrackbar("Low S", "Filtered Frame", &sLow, maxValueH, on_low_S_thresh_trackbar, this);
    createTrackbar("High S", "Filtered Frame", &sHigh, maxValueH, on_high_S_thresh_trackbar, this);
    createTrackbar("Low V", "Filtered Frame", &vLow, maxValueH, on_low_V_thresh_trackbar, this);
    createTrackbar("High V", "Filtered Frame", &vHigh, maxValueH, on_high_V_thresh_trackbar, this);
    createTrackbar("Sigma 1", "Filtered Frame", &sigma1, maxValueH, on_sigma_1_thresh_trackbar, this);
    createTrackbar("Sigma 2", "Filtered Frame", &sigma2, maxValueH, on_sigma_2_thresh_trackbar, this);

    while(true){

        cvtColor(frame, frameHSV, COLOR_BGR2HSV);

        DoGFilter(frameHSV, frameHSV);

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
    std::cout << "Sigma 1: " << sigma1 << std::endl;
    std::cout << "Sigma 2: " << sigma2 << std::endl;
    std::cout << "Finished Calibrating." << std::endl;
}