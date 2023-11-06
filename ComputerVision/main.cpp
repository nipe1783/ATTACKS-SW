#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

// Blob Detection Imports
#include "blobDetection/BlobDetection.h"
#include "visualizer/Visualizer.h"
#include "benchmarking/Benchmarking.h"
#include "scripts/Scripts.h"


using namespace cv;
int main()
{
    BlobDetection blobDetector;
    Scripts script;
    Scripts::VideoRunner("DroneTestFootage.mp4",blobDetector);
    // std::string imagePath = "../images/EORSSD/test-images/0004.jpg"; 
    // std::string imagePath = "../images/cars/train/DJI_0005-0018_jpg.rf.039f98afd951948ab4ccea83cadafacc.jpg";
    // std::string imagePath = "../images/cars/train/DJI_0013-0036_jpg.rf.955b80bd98684ce60bb92b3267c3004e.jpg";
    // std::string imagePath = "../images/EORSSD/test-images/0008.jpg";
    // std::string imagePath =  "../images/sheep/train/DJI_0004_0257_jpg.rf.a9a470104a8a36978a2fb2db0c3f9cd6.jpg";
    // for(double gamma = 0; gamma < 1; gamma += 0.1){
    //     cvtColor(frame, dst, COLOR_BGR2HSV);
    //     blobDetector.Mask(dst, dst, Scalar(0, 0, 182), Scalar(169, 19, 255));
    //     cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
    //     blobDetector.gammaCorrection(frame, dst, gamma);
    //     Visualizer::saveFrame(dst, "../output/gamma/" + std::to_string(gamma) + ".jpg");
    // }
    // for(int sigma2 = 1; sigma2 < 10; sigma2++){
    //     cvtColor(frame, dst, COLOR_BGR2HSV);
    //     blobDetector.Mask(dst, dst, Scalar(0, 0, 182), Scalar(169, 19, 255));
    //     cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
    //     blobDetector.DoGFilter(frame, dst, sigma1, sigma2);
    //     Visualizer::saveFrame(dst, "../output/sigma2/" + std::to_string(sigma2) + ".jpg");
    // }
    // for(double gamma = 0; gamma < 1; gamma += 0.1){
    //     for(int sigma2 = 1; sigma2 < 10; sigma2++) {
    //         for(int sigma1 = 1; sigma1 < sigma2; sigma1++) {
    //             cvtColor(frame, dst, COLOR_BGR2HSV);
    //             blobDetector.Mask(dst, dst, Scalar(0, 0, 182), Scalar(169, 19, 255));
    //             cv::cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
    //             blobDetector.gammaCorrection(dst, dst, gamma);
    //             blobDetector.DoGFilter(dst, dst, sigma1, sigma2);
    //             blobDetector.contrastEqualization(dst, dst, alpha, tau);
    //             Visualizer::saveFrame(dst, "../output/total/gamma: " + std::to_string(gamma) + "sigma1:" + std::to_string(sigma1) + "sigma2:" + std::to_string(sigma2) + ".jpg");
    //         }
    //     }
    // }
    
    int sigma1 = 1;
    int sigma2 = 2;
    double alpha = 0.5;
    double tau = 10;
    double areaThreshold = 150;
    double minError = MAXFLOAT;
    double currError = MAXFLOAT;
    cv::Mat frame;
    cv::Mat dst;

    // for(double gamma = 0; gamma < 1; gamma += 0.1){
    //     for(int sigma2 = 1; sigma2 < 10; sigma2++) {
    //         for(int sigma1 = 1; sigma1 < sigma2; sigma1++) {
    //             // for(int areaThreshold = 100; areaThreshold < 1000; areaThreshold += 5){
    //                 for(int lbChannel = 0; lbChannel <= 150; lbChannel += 5) {    // varying third channel of lowerBound
    //                     for(int ubChannel = 150; ubChannel <= 255; ubChannel += 5) {  // varying second channel of upperBound
    //                         cv::Scalar lowerBound = cv::Scalar(0, 0, lbChannel);
    //                         cv::Scalar upperBound = cv::Scalar(255, ubChannel, 255);

    //                         currError = Benchmarking::run1("../images/train1", "../images/label1", blobDetector, gamma, sigma1, sigma2, alpha, tau, areaThreshold, lowerBound, upperBound);

    //                         if(currError < minError){
    //                             minError = currError;
    //                             std::cout << "gamma: " << gamma << " sigma1: " << sigma1 << " sigma2: " << sigma2 << " lowerBound: " << lowerBound << " upperBound: " << upperBound << " minError: " << minError << std::endl;
    //                             cv::Mat frame = cv::imread("../images/train1/0004.jpg");
    //                             blobDetector.detect1(frame, dst, lowerBound, upperBound, gamma, sigma1, sigma2, alpha, tau, areaThreshold);
    //                             Visualizer::saveFrame(dst, "../output/total/min.jpg");
    //                         }
    //                         std::cout << "min error: " << minError << " curr error: " << currError << std::endl;
    //                     }
    //                 }
    //             // }
    //         }
    //     }
    // }

    return 0;
}