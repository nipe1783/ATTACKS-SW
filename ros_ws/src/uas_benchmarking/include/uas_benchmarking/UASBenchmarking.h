# pragma once
#include <iostream>
#include "uas_computer_vision/BlobDetector.h"

class UASBenchmarking
{
    public:
        UASBenchmarking() {};
        ~UASBenchmarking() {};
        static void run(const std::string& datasetPath, const std::string& datasetLabelsPath, BlobDetector& blobDetector);
};