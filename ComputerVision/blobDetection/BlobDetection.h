#pragma once
#include <string>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../blob/Blob.h"
#include <memory>

using namespace cv;

class BlobDetection
{
private:
    // Static callback methods for trackbars:
    static void on_low_H_thresh_trackbar(int, void*);
    static void on_high_H_thresh_trackbar(int, void*);
    static void on_low_S_thresh_trackbar(int, void*);
    static void on_high_S_thresh_trackbar(int, void*);
    static void on_low_V_thresh_trackbar(int, void*);
    static void on_high_V_thresh_trackbar(int, void*);

public:
    // Constructor and destructor:
    BlobDetection() = default;
    ~BlobDetection() = default;

    // Public methods:
    void calibrate(Mat frame);
    std::unique_ptr<Blob> detect(const Mat& frame);

    /**
     * @brief Performs gamma correction on the given frame. Source:https://ieeexplore.ieee.org/document/5411802
     * 
     * @param frame The frame to perform gamma correction on.
     * @param gamma The gamma value to use. [0, 1]
     **/
    void gammaCorrection(Mat& frame, Mat& dst, double gamma);

    void DoGFilter(Mat& frame, Mat& dst, int sigma1, int sigma2);

    void Mask(Mat& frame, Mat& dst, Scalar lowerBound, Scalar upperBound);

    void contrastEqualization(Mat& frame, Mat& dst, double alpha, double tau);

    void detect1(Mat& frame, Mat& dst, Scalar lowerBound, Scalar upperBound, float gamma, int sigma1, int sigma2, double alpha, double tau, double areaThreshold);
    void simpleDetect(Mat& frame, Mat& dst);
    
    // Member variables:
    int hLow = 61;
    int hHigh = 141;
    int sLow = 29;
    int sHigh = 131;
    int vLow = 82;
    int vHigh = 203;
    int blurSize = 20;
    int areaThreshold = 300;

    
    static const int maxValueH = 255;
    static const int maxValue = 255;
};