#include "uas_phases/UASCoarseLocalizationPhase.h"
#include <cmath>
#include <Eigen/Dense>
#include "uas_helpers/RGVState.h"
#include "uas_helpers/RGV.h"

UASCoarseLocalizationPhase::UASCoarseLocalizationPhase()
{
    phaseName_ = "coarse";
    alpha_ = 0;
    REnu_ << 1, 0, 0,
             0, -1, 0,
             0, 0, -1;
}

UASState UASCoarseLocalizationPhase::generateDesiredState(CVImg cvImg, UASState uasState)
{   
    UASState desiredUASState;
    desiredUASState.bxV_ = 0.0f;
    desiredUASState.byV_ = 0.0f;
    desiredUASState.bzV_ = 0.0f;
    float bodyX = 0.0f;
    float bodyY = 0.0f;

    Blob blob = cvImg.blobs[0];
    if (std::abs(blob.x - cvImg.centerX) / static_cast<float>(cvImg.width) > tolerance_) {
        bodyX = (blob.x - cvImg.centerX) / static_cast<float>(cvImg.width) * 3;
    }

    if (std::abs(blob.y - cvImg.centerY) / static_cast<float>(cvImg.height) > tolerance_) {
        bodyY = (blob.y - cvImg.centerY) / static_cast<float>(cvImg.height) * 3;
    }

    desiredUASState.bxV_ = bodyX * cos(uasState.ipsi_  + M_PI_2) - bodyY * sin(uasState.ipsi_ + M_PI_2);
    desiredUASState.byV_ = bodyX * sin(uasState.ipsi_  + M_PI_2) + bodyY * cos(uasState.ipsi_ + M_PI_2);

    desiredUASState.bzV_ = (desiredAltitude_ - uasState.iz_) * kpZ_;
    return desiredUASState;
}