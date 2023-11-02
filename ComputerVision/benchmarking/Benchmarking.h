# pragma once
#include <iostream>
#include "../blobDetection/BlobDetection.h"

class Benchmarking
{
    public:
        Benchmarking();
        ~Benchmarking();

        static double run1(std::string datasetPath, std::string datasetLabelsPath, BlobDetection blobDetector, double gamma, int sigma1, int sigma2, double alpha, double tau, double areaThreshold, cv::Scalar lowerBound, cv::Scalar upperBound);
        static void runSimple(std::string datasetPath, BlobDetection blobDetector);
};