#include "uas_computer_vision/BlobDetector.h"

void BlobDetector::on_low_H_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->hLow_ = min(instance->hHigh_-1, instance->hLow_);
    setTrackbarPos("Low H", "Filtered Frame", instance->hLow_);
}

void BlobDetector::on_high_H_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->hHigh_ = max(instance->hHigh_, instance->hLow_+1);
    setTrackbarPos("High H", "Filtered Frame", instance->hHigh_);
}

void BlobDetector::on_low_S_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->sLow_ = min(instance->sHigh_-1, instance->sLow_);
    setTrackbarPos("Low S", "Filtered Frame", instance->sLow_);
}

void BlobDetector::on_high_S_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->sHigh_ = max(instance->sHigh_, instance->sLow_+1);
    setTrackbarPos("High S", "Filtered Frame", instance->sHigh_);
}

void BlobDetector::on_low_V_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->vLow_= min(instance->vHigh_-1, instance->vLow_);
    setTrackbarPos("Low V", "Filtered Frame", instance->vLow_);
}

void BlobDetector::on_high_V_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->vHigh_= max(instance->vHigh_, instance->vLow_+1);
    setTrackbarPos("High V", "Filtered Frame", instance->vHigh_);
}

void BlobDetector::on_area_threshold_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    setTrackbarPos("Area Threshold", "Filtered Frame", instance->areaThreshold_);
}