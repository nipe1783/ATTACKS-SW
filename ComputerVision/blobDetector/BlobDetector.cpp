// #include "BlobDetector.h"
// #include "../blob/Blob.h"
// #include <opencv2/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <opencv2/opencv.hpp>
// #include <memory>
// #include <cmath>
// #include "../visualizer/Visualizer.h"

// using namespace cv;

// void BlobDetection::gammaCorrection(Mat& frame, Mat& dst, double gamma)
// {
//     CV_Assert(gamma >= 0);
//     Mat lookupTable(1, 256, CV_8U);
//     uchar* p = lookupTable.ptr();

//     if(gamma > 0){
//         for(int i = 0; i < 256; ++i){
//             p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
//         }
//     }
//     else{
//         for(int i = 0; i < 256; ++i){
//             p[i] = saturate_cast<uchar>(log(i + 1) / log(256) * 255.0);
//         }
//     }

//     LUT(frame, lookupTable, dst);
// }

// void BlobDetection::DoGFilter(Mat& frame, Mat& dst, int sigma1, int sigma2){
//     Mat gaussian1, gaussian2;
//     GaussianBlur(frame, gaussian1, Size(), sigma1, sigma1); // possibly change K size in the future for better performance
//     GaussianBlur(frame, gaussian2, Size(), sigma2, sigma2);
//     subtract(gaussian1, gaussian2, dst);
// }

// void BlobDetection::Mask(Mat& frame, Mat& dst, Scalar lowerBound, Scalar upperBound) {
//     // Create the mask for the pixels within the given bounds
//     Mat mask;
//     inRange(frame, lowerBound, upperBound, mask);

//     // Calculate the average color of the frame
//     Scalar avgColor = mean(frame);

//     // Initialize dst with the original frame
//     frame.copyTo(dst);

//     // Create a matrix with the average color
//     Mat avgColorMat(frame.size(), frame.type(), avgColor);

//     // Set pixels where the mask is not white (outside the range) to the average color in dst
//     avgColorMat.copyTo(dst, ~mask);
// }

// void BlobDetection::contrastEqualization(Mat& frame, Mat& dst, double alpha, double tau) {

//     // Convert image to float type for processing
//     int originalType = frame.type();
//     frame.convertTo(frame, CV_32FC1);

//     // Stage 1
//     Mat imgRaisedToAlpha;
//     pow(frame, alpha, imgRaisedToAlpha);
//     double mean1 = mean(abs(imgRaisedToAlpha))[0];
//     frame = frame / std::pow(mean1, 1.0 / alpha);

//     // Stage 2
//     Mat imgAbs = abs(frame);
//     Mat minImg = min(imgAbs, tau * Mat::ones(frame.size(), frame.type()));
//     pow(minImg, alpha, minImg);
//     double mean2 = mean(minImg)[0];
//     frame = frame / std::pow(mean2, 1.0 / alpha);

//     // Convert the image back to its original type
//     if (originalType == CV_8U) {
//         double minVal, maxVal;
//         minMaxLoc(frame, &minVal, &maxVal);
//         frame.convertTo(dst, originalType, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
//     } else {
//         frame.convertTo(dst, originalType);
//     }

// }

// void BlobDetection::calibrate(Mat frame)
// {
//     std::cout << "Calibrating..." << std::endl;
//     Mat frameHSV;
//     Mat filteredFrame;

//     namedWindow("Original Frame", WINDOW_AUTOSIZE);
//     namedWindow("Filtered Frame", WINDOW_AUTOSIZE);
    
//     imshow("Original Frame", frame);
    
//     cvtColor(frame, frameHSV, COLOR_BGR2HSV);
//     imshow("Filtered Frame", frameHSV);
//     createTrackbar("Low H", "Filtered Frame", &hLow, maxValueH, on_low_H_thresh_trackbar, this);
//     createTrackbar("High H", "Filtered Frame", &hHigh, maxValueH, on_high_H_thresh_trackbar, this);
//     createTrackbar("Low S", "Filtered Frame", &sLow, maxValueH, on_low_S_thresh_trackbar, this);
//     createTrackbar("High S", "Filtered Frame", &sHigh, maxValueH, on_high_S_thresh_trackbar, this);
//     createTrackbar("Low V", "Filtered Frame", &vLow, maxValueH, on_low_V_thresh_trackbar, this);
//     createTrackbar("High V", "Filtered Frame", &vHigh, maxValueH, on_high_V_thresh_trackbar, this);
//     createTrackbar("Blur Size", "Filtered Frame", &blurSize, 100, NULL);

//     while(true){

//         cvtColor(frame, frameHSV, COLOR_BGR2HSV);

//         // Blur the frame
//         if(blurSize > 0 && blurSize % 2 == 1) {
//             GaussianBlur(frameHSV, frameHSV, Size(blurSize, blurSize), 0);
//         }
//         else if(blurSize > 0){
//             blurSize++;
//             GaussianBlur(frameHSV, frameHSV, Size(blurSize, blurSize), 0);
//         }

//         // Detect the object based on HSV Range Values
//         inRange(frameHSV, Scalar(hLow, sLow, vLow), Scalar(hHigh, sHigh, vHigh), filteredFrame);
        
//         // Show the filtered frame
//         imshow("Filtered Frame", filteredFrame);

//         char key = (char) waitKey(30);
//         if (key == 'q' || key == 27) // 'q' or 'ESC' key
//         {
//             break;
//         }
//     }
//     std::cout << "Low H: " << hLow << std::endl;
//     std::cout << "High H: " << hHigh << std::endl;
//     std::cout << "Low S: " << sLow << std::endl;
//     std::cout << "High S: " << sHigh << std::endl;
//     std::cout << "Low V: " << vLow << std::endl;
//     std::cout << "High V: " << vHigh << std::endl;
//     std::cout << "Blur Size: " << blurSize << std::endl;
//     std::cout << "Finished Calibrating." << std::endl;
// }


// std::unique_ptr<Blob> BlobDetection::detect(const Mat& frame)
// {
//     Mat frameHSV, filteredFrame, labels, stats, centroids;

//     // Filtering image based on calibration values
//     cvtColor(frame, frameHSV, COLOR_BGR2HSV);
//     GaussianBlur(frameHSV, frameHSV, Size(blurSize, blurSize), 0);
//     inRange(frameHSV, Scalar(hLow, sLow, vLow), Scalar(hHigh, sHigh, vHigh), filteredFrame);

//     // Detecting blobs
//     int numberOfLabels = connectedComponentsWithStats(filteredFrame, labels, stats, centroids);
//     int largestBlobLabel = 0;
//     int maxArea = 0;
//     for (int i = 1; i < numberOfLabels; i++) { // Start from 1 to skip background
//         int area = stats.at<int>(i, cv::CC_STAT_AREA);
//         if(area > maxArea) {
//             maxArea = area;
//             largestBlobLabel = i;
//         }
//     }

//     // bounding largest blob
//     if (largestBlobLabel != 0) { 
//         int x = stats.at<int>(largestBlobLabel, cv::CC_STAT_LEFT);
//         int y = stats.at<int>(largestBlobLabel, cv::CC_STAT_TOP);
//         int width = stats.at<int>(largestBlobLabel, cv::CC_STAT_WIDTH);
//         int height = stats.at<int>(largestBlobLabel, cv::CC_STAT_HEIGHT);
//         int area = stats.at<int>(largestBlobLabel, cv::CC_STAT_AREA);
//         std::unique_ptr<Blob> blobPtr = std::make_unique<Blob>(x, y, width, height, area);
//         return blobPtr;
//     }
//     return nullptr;
// }

// void BlobDetection::simpleDetect(Mat& frame, Mat& dst){
//     Mat frameHSV, labels, stats, centroids;

//     // Filtering image based on calibration values
//     cvtColor(frame, dst, COLOR_BGR2HSV);
//     GaussianBlur(dst, dst, Size(blurSize, blurSize), 0);
//     inRange(dst, Scalar(hLow, sLow, vLow), Scalar(hHigh, sHigh, vHigh), dst);

//     // Detecting blobs
//     int numberOfLabels = connectedComponentsWithStats(dst, labels, stats, centroids);
//     int areaThreshold = 150;

//     for (int i = 1; i < numberOfLabels; i++) { // Start from 1 to skip background
//         int area = stats.at<int>(i, cv::CC_STAT_AREA);
//         std::cout << "area: " << area << std::endl;

//         if(area > areaThreshold) {
//             int x = stats.at<int>(i, cv::CC_STAT_LEFT);
//             int y = stats.at<int>(i, cv::CC_STAT_TOP);
//             int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
//             int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
//             cv::Rect bounding_box = cv::Rect(x, y, width, height);
//             cv::rectangle(frame, bounding_box, cv::Scalar(255, 0, 0), 2);
//         }
//     }

//     Visualizer::twoFrame(frame, dst);
//     waitKey(0);
// }

// void BlobDetection::detect1(Mat& frame, Mat& dst, cv::Scalar lowerBound, cv::Scalar upperBound, float gamma, int sigma1, int sigma2, double alpha, double tau, double areaThreshold){
//     cv::Mat frameHSV;
//     cvtColor(frame, dst, COLOR_BGR2HSV);
//     Mask(dst, dst, lowerBound, upperBound);
//     cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR);
//     cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
//     gammaCorrection(dst, dst, gamma);
//     DoGFilter(dst, dst, sigma1, sigma2);
//     contrastEqualization(dst, dst, alpha, tau);
//     cv::threshold(dst, dst, 100, 255, cv::THRESH_BINARY);
//     std::vector<std::vector<cv::Point>> contours;
//     std::vector<cv::Vec4i> hierarchy;
//     cv::findContours(dst, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
//     double aspectRatioThreshold = 20.0;
//     for (size_t i = 0; i < contours.size(); i++) {
//         // Approximate contour with a polygon
//         if(cv::contourArea(contours[i]) > areaThreshold){
//             cv::Rect bounding_box = cv::boundingRect(contours[i]);
//             double aspectRatio = static_cast<double>(bounding_box.width) / bounding_box.height;

//             // Check if the blob is not too thin and long
//             if (aspectRatio < aspectRatioThreshold && aspectRatio > 1.0/aspectRatioThreshold) {
//                 cv::rectangle(frame, bounding_box, cv::Scalar(255, 0, 0), 2);
//             }
//         }
//     }
// }

// void BlobDetection::on_low_H_thresh_trackbar(int pos, void* userdata)
// {
//     BlobDetection* instance = (BlobDetection*)userdata;
//     instance->hLow = min(instance->hHigh-1, instance->hLow);
//     setTrackbarPos("Low H", "Filtered Frame", instance->hLow);
// }

// void BlobDetection::on_high_H_thresh_trackbar(int pos, void* userdata)
// {
//     BlobDetection* instance = (BlobDetection*)userdata;
//     instance->hHigh = max(instance->hHigh, instance->hLow+1);
//     setTrackbarPos("High H", "Filtered Frame", instance->hHigh);
// }

// void BlobDetection::on_low_S_thresh_trackbar(int pos, void* userdata)
// {
//     BlobDetection* instance = (BlobDetection*)userdata;
//     instance->sLow = min(instance->sHigh-1, instance->sLow);
//     setTrackbarPos("Low S", "Filtered Frame", instance->sLow);
// }

// void BlobDetection::on_high_S_thresh_trackbar(int pos, void* userdata)
// {
//     BlobDetection* instance = (BlobDetection*)userdata;
//     instance->sHigh = max(instance->sHigh, instance->sLow+1);
//     setTrackbarPos("High S", "Filtered Frame", instance->sHigh);
// }

// void BlobDetection::on_low_V_thresh_trackbar(int pos, void* userdata)
// {
//     BlobDetection* instance = (BlobDetection*)userdata;
//     instance->vLow = min(instance->vHigh-1, instance->vLow);
//     setTrackbarPos("Low V", "Filtered Frame", instance->vLow);
// }

// void BlobDetection::on_high_V_thresh_trackbar(int pos, void* userdata)
// {
//     BlobDetection* instance = (BlobDetection*)userdata;
//     instance->vHigh = max(instance->vHigh, instance->vLow+1);
//     setTrackbarPos("High V", "Filtered Frame", instance->vHigh);
// }