#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "computer_vision/Visualizer.h"

void Visualizer::twoFrame(const cv::Mat& frame1,const cv::Mat& frame2)
{
    if (frame1.empty() || frame2.empty()) {
        std::cout << "Could not open or find the images!" << std::endl;
        return;
    }
    
    if (frame1.rows != frame2.rows) {
        double scale = (double) frame1.rows / frame2.rows;
        cv::resize(frame2, frame2, cv::Size(), scale, scale);
    }

    cv::Mat frame1_rgb, frame2_rgb;

    // Convert frame1 to RGB
    if (frame1.channels() == 1) {
        cv::cvtColor(frame1, frame1_rgb, cv::COLOR_GRAY2BGR);
    } else {
        frame1_rgb = frame1.clone();
    }

    // Convert frame2 to RGB
    if (frame2.channels() == 1) {
        cv::cvtColor(frame2, frame2_rgb, cv::COLOR_GRAY2BGR);
    } else {
        frame2_rgb = frame2.clone();
    }

    cv::Mat combined_image;
    cv::hconcat(frame1_rgb, frame2_rgb, combined_image);
    cv::imshow("Combined Image", combined_image);
}

void Visualizer::saveFrame(const cv::Mat& frame1, std::string path){
    if (frame1.empty()) {
        std::cout << "Could not open or find the image!" << std::endl;
        return;
    }
    cv::Mat frame1_rgb;
    if (frame1.channels() == 1) {
        cv::cvtColor(frame1, frame1_rgb, cv::COLOR_GRAY2BGR);
    } else {
        frame1_rgb = frame1.clone();
    }
    cv::imwrite(path, frame1_rgb);
}

void Visualizer::saveTwoFrame(const cv::Mat& frame1,const cv::Mat& frame2, std::string path)
{
    if (frame1.empty() || frame2.empty()) {
        std::cout << "Could not open or find the images!" << std::endl;
        return;
    }
    
    if (frame1.rows != frame2.rows) {
        double scale = (double) frame1.rows / frame2.rows;
        cv::resize(frame2, frame2, cv::Size(), scale, scale);
    }

    cv::Mat frame1_rgb, frame2_rgb;

    // Convert frame1 to RGB
    if (frame1.channels() == 1) {
        cv::cvtColor(frame1, frame1_rgb, cv::COLOR_GRAY2BGR);
    } else {
        frame1_rgb = frame1.clone();
    }

    // Convert frame2 to RGB
    if (frame2.channels() == 1) {
        cv::cvtColor(frame2, frame2_rgb, cv::COLOR_GRAY2BGR);
    } else {
        frame2_rgb = frame2.clone();
    }

    cv::Mat combined_image;
    cv::hconcat(frame1_rgb, frame2_rgb, combined_image);
    cv::imwrite(path, combined_image);
}

