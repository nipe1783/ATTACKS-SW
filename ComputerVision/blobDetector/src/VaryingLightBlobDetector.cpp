#include "../VaryingLightBlobDetector.h"
#include "../../visualizer/Visualizer.h"

using namespace cv;

void VaryingLightBlobDetector::detect(Mat& frame, Mat& dst){
    cv::Mat frameHSV;
    cvtColor(frame, dst, COLOR_BGR2HSV);
    Mask(dst, dst);
    cv::cvtColor(dst, dst, cv::COLOR_HSV2BGR);
    cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
    gammaCorrection(dst, dst);
    DoGFilter(dst, dst);
    contrastEqualization(dst, dst);
    cv::threshold(dst, dst, 100, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(dst, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    double aspectRatioThreshold = 20.0;
    for (size_t i = 0; i < contours.size(); i++) {
        // Approximate contour with a polygon
        if(cv::contourArea(contours[i]) > areaThreshold){
            cv::Rect bounding_box = cv::boundingRect(contours[i]);
            cv::rectangle(frame, bounding_box, cv::Scalar(255, 0, 0), 2);
        }
    }
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
    createTrackbar("Low H", "Filtered Frame", &hLow, maxValueH, on_low_H_thresh_trackbar, this);
    createTrackbar("High H", "Filtered Frame", &hHigh, maxValueH, on_high_H_thresh_trackbar, this);
    createTrackbar("Low S", "Filtered Frame", &sLow, maxValueH, on_low_S_thresh_trackbar, this);
    createTrackbar("High S", "Filtered Frame", &sHigh, maxValueH, on_high_S_thresh_trackbar, this);
    createTrackbar("Low V", "Filtered Frame", &vLow, maxValueH, on_low_V_thresh_trackbar, this);
    createTrackbar("High V", "Filtered Frame", &vHigh, maxValueH, on_high_V_thresh_trackbar, this);
    createTrackbar("Sigma 1", "Filtered Frame", &sigma1, maxValueH, on_sigma_1_thresh_trackbar, this);
    createTrackbar("Sigma 2", "Filtered Frame", &sigma2, maxValueH, on_sigma_2_thresh_trackbar, this);
    createTrackbar("Alpha", "Filtered Frame", &alphaSlider, maxValueAlphaSlider, on_alpha_trackbar, this);
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
    inRange(frame, Scalar(hLow, sLow, vLow), Scalar(hHigh, sHigh, vHigh), mask);

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

void VaryingLightBlobDetector::on_sigma_1_thresh_trackbar(int pos, void* userdata)
{
    VaryingLightBlobDetector* instance = (VaryingLightBlobDetector*)userdata;
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

