#pragma once
#include <vector>
#include "computer_vision/Blob.h"


class CVImg
{
    public:
        // methods
        CVImg(int width, int height, int centerX, int centerY, std::vector<Blob> blobs) : width(width), height(height), centerX(centerX), centerY(centerY), blobs(blobs) { };
        CVImg() {};
        ~CVImg() {};

        // fields
        int width;
        int height;
        int centerX;
        int centerY;
        std::vector<Blob> blobs;
};