# pragma once
#include <iostream>
#include "../blobDetector/BlobDetector.h"
#include "../blobDetector/BasicBlobDetector.h"
#include "../blobDetector/VaryingLightBlobDetector.h"

class Benchmarking
{
    public:
        Benchmarking();
        ~Benchmarking();
        static void runBasic(const std::string& datasetPath, const std::string& datasetLabelsPath, BasicBlobDetector& blobDetector);
        static void runVarying(const std::string& datasetPath, const std::string& datasetLabelsPath, VaryingLightBlobDetector& blobDetector);
};