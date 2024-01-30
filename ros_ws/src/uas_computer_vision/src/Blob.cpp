#include "uas_computer_vision/Blob.h"

Blob::Blob(int x, int y, int width, int height, int area)
{
    this->x = x;
    this->y = y;
    this->width = width;
    this->height = height;
    this->area = area;
}

Blob::~Blob()
{
}
