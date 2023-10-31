#include "Blob.h"

Blob::Blob(int x, int y, int width, int height, int area)
{
    this->x = x;
    this->y = y;
    this->width = width;
    this->height = height;
    this->area = area;
}

Blob::Blob()
{
    this->x = 0;
    this->y = 0;
    this->width = 0;
    this->height = 0;
    this->area = 0;
}

Blob::~Blob()
{
}
