#pragma once
#include <string>
#include <iostream>

class BlobDetection
{
    private:
        /* data */
    public:
        void calibrate(std::string imagePath);
        void detect(std::string imagePath);
        BlobDetection();
        ~BlobDetection();

};