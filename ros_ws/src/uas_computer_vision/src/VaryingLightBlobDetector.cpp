#include "uas_computer_vision/VaryingLightBlobDetector.h"
#include "uas_computer_vision/Visualizer.h"
#include "uas_computer_vision/Blob.h"
#include <vector>

using namespace cv;

CVImg VaryingLightBlobDetector::detect(const Mat& frame){
    Mat dst;
    std::vector<Blob> myblobVector; // Create an empty blob vector

    cvtColor(frame, dst, COLOR_BGR2HSV);
    Mask(dst, dst);
    cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR);
    cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
    gammaCorrection(dst, dst);
    DoGFilter(dst, dst);
    contrastEqualization(dst, dst);
    cv::GaussianBlur(dst, dst, cv::Size(blurSize_, blurSize_), 0, 0);
    cv::threshold(dst, dst, intensityThreshold, 255, cv::THRESH_BINARY);
    Mat element = getStructuringElement(cv::MORPH_RECT,
        cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
        cv::Point(dilationSize, dilationSize));
    cv::dilate(dst, dst, element);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dst.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    double maxArea = 0;
    int maxAreaContourIndex = -1;
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
        myblobVector.push_back(Blob(boundingBox.x, boundingBox.y, boundingBox.width, boundingBox.height, boundingBox.area()));
    }
    return CVImg(frame.cols, frame.rows, frame.cols / 2, frame.rows / 2, myblobVector);
}

std::vector<Blob> VaryingLightBlobDetector::detect(Mat& frame, Mat& dst){
    std::vector<Blob> myblobVector; // Create an empty blob vector

    cvtColor(frame, dst, COLOR_BGR2HSV);
    Mask(dst, dst);
    cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR);
    cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
    gammaCorrection(dst, dst);
    DoGFilter(dst, dst);
    contrastEqualization(dst, dst);
    cv::GaussianBlur(dst, dst, cv::Size(blurSize_, blurSize_), 0, 0);
    cv::threshold(dst, dst, intensityThreshold, 255, cv::THRESH_BINARY);
    Mat element = getStructuringElement(cv::MORPH_RECT,
        cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
        cv::Point(dilationSize, dilationSize));
    cv::dilate(dst, dst, element);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dst.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    double maxArea = 0;
    int maxAreaContourIndex = -1;
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
        myblobVector.push_back(Blob(boundingBox.x, boundingBox.y, boundingBox.width, boundingBox.height, boundingBox.area()));
    }
    return myblobVector;
}

void VaryingLightBlobDetector::calibrate(Mat& frame){
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
    createTrackbar("Sigma 1", "Filtered Frame", &sigma1, maxValueH_, on_sigma_1_thresh_trackbar, this);
    createTrackbar("Sigma 2", "Filtered Frame", &sigma2, maxValueH_, on_sigma_2_thresh_trackbar, this);
    createTrackbar("Intensity Threshold", "Filtered Frame", &intensityThreshold, maxValueH_, on_intensity_thresh_trackbar, this);
    createTrackbar("Alpha", "Filtered Frame", &alphaSlider, maxValueAlphaSlider, on_alpha_trackbar, this);
    createTrackbar("Blur Size", "Filtered Frame", &blurSize_, 100, NULL);
    createTrackbar("Dilation Size", "Filtered Frame", &dilationSize, 100, NULL);
    createTrackbar("Area Threshold", "Filtered Frame", &areaThreshold_, 1000, NULL);

    while(true){

        Mat dst;
        cvtColor(frame, dst, COLOR_BGR2HSV);
        Mask(dst, dst);
        
        // Show the filtered frame
        imshow("Filtered Frame", dst);

        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }

    while(true){

        Mat dst;
        cvtColor(frame, dst, COLOR_BGR2HSV);
        Mask(dst, dst);
        cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR);
        cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
        gammaCorrection(dst, dst);
        DoGFilter(dst, dst);
        contrastEqualization(dst, dst);
        
        // Show the filtered frame
        imshow("Filtered Frame", dst);

        char key = (char) waitKey(30);
        if (key == 'q' || key == 27) // 'q' or 'ESC' key
        {
            break;
        }
    }

    while(true) {
        if(blurSize_ % 2 == 0) {
            blurSize_++;
        }
        Mat dst;
        cvtColor(frame, dst, COLOR_BGR2HSV);
        Mask(dst, dst);
        cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR);
        cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
        gammaCorrection(dst, dst);
        DoGFilter(dst, dst);
        contrastEqualization(dst, dst);
        cv::GaussianBlur(dst, dst, cv::Size(blurSize_, blurSize_), 0, 0);
        cv::threshold(dst, dst, intensityThreshold, 255, cv::THRESH_BINARY);
        Mat element = getStructuringElement(cv::MORPH_RECT,
            cv::Size(2 * dilationSize + 1, 2 * dilationSize + 1),
            cv::Point(dilationSize, dilationSize));
        cv::dilate(dst, dst, element);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(dst.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        Mat mask = Mat::zeros(dst.size(), dst.type());
        for(size_t i = 0; i < contours.size(); i++) {
            if(cv::contourArea(contours[i]) > areaThreshold_) {
                drawContours(mask, contours, static_cast<int>(i), Scalar(255), cv::FILLED);
            }
        }
        dst = dst & mask;
        imshow("Filtered Frame", dst);
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

void VaryingLightBlobDetector::gammaCorrection(Mat& frame, Mat& dst)
{
    CV_Assert(gamma >= 0);
    Mat lookupTable(1, 256, CV_8U);
    uchar* p = lookupTable.ptr();

    if(gamma > 0){
        for(int i = 0; i < 256; ++i){
            p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
        }
    }
    else{
        for(int i = 0; i < 256; ++i){
            p[i] = saturate_cast<uchar>(log(i + 1) / log(256) * 255.0);
        }
    }
    LUT(frame, lookupTable, dst);
}

void VaryingLightBlobDetector::DoGFilter(Mat& frame, Mat& dst){
    Mat gaussian1, gaussian2;
    GaussianBlur(frame, gaussian1, Size(), sigma1, sigma1); // possibly change K size in the future for better performance
    GaussianBlur(frame, gaussian2, Size(), sigma2, sigma2);
    subtract(gaussian1, gaussian2, dst);
}

void VaryingLightBlobDetector::Mask(Mat& frame, Mat& dst) {
    // Create the mask for the pixels within the given bounds
    Mat mask;
    inRange(frame, Scalar(hLow_, sLow_, vLow_), Scalar(hHigh_, sHigh_, vHigh_), mask);

    // Calculate the average color of the frame
    Scalar avgColor = mean(frame);

    // Initialize dst with the original frame
    frame.copyTo(dst);

    // Create a matrix with the average color
    Mat avgColorMat(frame.size(), frame.type(), avgColor);

    // Set pixels where the mask is not white (outside the range) to the average color in dst
    avgColorMat.copyTo(dst, ~mask);
}

void VaryingLightBlobDetector::contrastEqualization(Mat& frame, Mat& dst) {

    // Convert image to float type for processing
    int originalType = frame.type();
    frame.convertTo(frame, CV_32FC1);

    // Stage 1
    Mat imgRaisedToAlpha;
    pow(frame, alpha, imgRaisedToAlpha);
    double mean1 = mean(abs(imgRaisedToAlpha))[0];
    frame = frame / std::pow(mean1, 1.0 / alpha);

    // Stage 2
    Mat imgAbs = abs(frame);
    Mat minImg = min(imgAbs, tau * Mat::ones(frame.size(), frame.type()));
    pow(minImg, alpha, minImg);
    double mean2 = mean(minImg)[0];
    frame = frame / std::pow(mean2, 1.0 / alpha);

    // Convert the image back to its original type
    if (originalType == CV_8U) {
        double minVal, maxVal;
        minMaxLoc(frame, &minVal, &maxVal);
        frame.convertTo(dst, originalType, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    } else {
        frame.convertTo(dst, originalType);
    }

}

std::vector<Rect> VaryingLightBlobDetector::mergeNearbyContours(const std::vector<Rect>& boundingBoxes, float mergeThreshold) {
    std::vector<cv::Rect> mergedBoxes;
    for (const auto& box : boundingBoxes) {
        bool merged = false;
        for (auto& mergedBox : mergedBoxes) {
            if ((abs(mergedBox.x - box.x) < mergeThreshold && abs(mergedBox.y - box.y) < mergeThreshold) ||
                (mergedBox & box).area() > 0) {
                mergedBox |= box;
                merged = true;
                break;
            }
        }
        if (!merged) {
            mergedBoxes.push_back(box);
        }
    }
    return mergedBoxes;
}


void VaryingLightBlobDetector::on_sigma_1_thresh_trackbar(int pos, void* userdata)
{   
    VaryingLightBlobDetector* instance = (VaryingLightBlobDetector*)userdata;
    if (pos <= 0) {
        pos = 1;
    }
    instance->sigma1 = min(instance->sigma2-1, instance->sigma1);
    setTrackbarPos("Sigma 1", "Filtered Frame", instance->sigma1);
}

void VaryingLightBlobDetector::on_sigma_2_thresh_trackbar(int pos, void* userdata)
{
    VaryingLightBlobDetector* instance = (VaryingLightBlobDetector*)userdata;
    instance->sigma2 = max(instance->sigma2, instance->sigma1+1);
    setTrackbarPos("Sigma 2", "Filtered Frame", instance->sigma2);
}

void VaryingLightBlobDetector::on_alpha_trackbar(int pos, void* userdata)
{
    VaryingLightBlobDetector* instance = (VaryingLightBlobDetector*)userdata;
    instance->alpha = (double)pos / maxValueAlphaSlider;
}

void VaryingLightBlobDetector::on_intensity_thresh_trackbar(int pos, void* userdata)
{
    VaryingLightBlobDetector* instance = (VaryingLightBlobDetector*)userdata;
    instance->intensityThreshold = pos;
}
