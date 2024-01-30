#include "uas_phases/UASTrailingPhase.h"
#include <cmath>


UASTrailingPhase::UASTrailingPhase()
{
    phaseName_ = "Trailing";
}

UASState UASTrailingPhase::generateDesiredState(CVImg cvImg, UASState uasState)
{   
    UASState desiredUASState;
    desiredUASState.bxV = 0.0f;
    desiredUASState.byV = 0.0f;
    desiredUASState.bzV = 0.0f;
    float bodyX = 0.0f;
    float bodyY = 0.0f;

    Blob blob = cvImg.blobs[0];
    if (std::abs(blob.x - cvImg.centerX) / static_cast<float>(cvImg.width) > tolerance_) {
        bodyX = (blob.x - cvImg.centerX) / static_cast<float>(cvImg.width) * 3;
    }

    if (std::abs(blob.y - cvImg.centerY) / static_cast<float>(cvImg.height) > tolerance_) {
        bodyY = (blob.y - cvImg.centerY) / static_cast<float>(cvImg.height) * 3;
    }

    desiredUASState.bxV = bodyX * cos(uasState.ipsi  + M_PI_2) - bodyY * sin(uasState.ipsi + M_PI_2);
    desiredUASState.byV = bodyX * sin(uasState.ipsi  + M_PI_2) + bodyY * cos(uasState.ipsi + M_PI_2);
    return desiredUASState;
}