# pragma once
#include <iostream>
#include "../blobDetector/BlobDetector.h"

class Benchmarking
{
    public:
        Benchmarking();
        ~Benchmarking();
        static void run(const std::string& datasetPath, const std::string& datasetLabelsPath, BlobDetector& blobDetector);
};