#include "../BlobDetector.h"

void BlobDetector::on_low_H_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->hLow = min(instance->hHigh-1, instance->hLow);
    setTrackbarPos("Low H", "Filtered Frame", instance->hLow);
}

void BlobDetector::on_high_H_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->hHigh = max(instance->hHigh, instance->hLow+1);
    setTrackbarPos("High H", "Filtered Frame", instance->hHigh);
}

void BlobDetector::on_low_S_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->sLow = min(instance->sHigh-1, instance->sLow);
    setTrackbarPos("Low S", "Filtered Frame", instance->sLow);
}

void BlobDetector::on_high_S_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->sHigh = max(instance->sHigh, instance->sLow+1);
    setTrackbarPos("High S", "Filtered Frame", instance->sHigh);
}

void BlobDetector::on_low_V_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->vLow = min(instance->vHigh-1, instance->vLow);
    setTrackbarPos("Low V", "Filtered Frame", instance->vLow);
}

void BlobDetector::on_high_V_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->vHigh = max(instance->vHigh, instance->vLow+1);
    setTrackbarPos("High V", "Filtered Frame", instance->vHigh);
}

void BlobDetector::on_area_threshold_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    setTrackbarPos("Area Threshold", "Filtered Frame", instance->areaThreshold);
}

void BlobDetector::on_sigma_1_thresh_trackbar(int pos, void* userdata)
{   
    BlobDetector* instance = (BlobDetector*)userdata;
    if (pos <= 0) {
        pos = 1;
    }
    instance->sigma1 = min(instance->sigma2-1, instance->sigma1);
    setTrackbarPos("Sigma 1", "Filtered Frame", instance->sigma1);
}

void BlobDetector::on_sigma_2_thresh_trackbar(int pos, void* userdata)
{
    BlobDetector* instance = (BlobDetector*)userdata;
    instance->sigma2 = max(instance->sigma2, instance->sigma1+1);
    setTrackbarPos("Sigma 2", "Filtered Frame", instance->sigma2);
}

void BlobDetector::DoGFilter(Mat& frame, Mat& dst){
    Mat gaussian1, gaussian2;
    GaussianBlur(frame, gaussian1, Size(), sigma1, sigma1); // possibly change K size in the future for better performance
    GaussianBlur(frame, gaussian2, Size(), sigma2, sigma2);
    subtract(gaussian1, gaussian2, dst);
}