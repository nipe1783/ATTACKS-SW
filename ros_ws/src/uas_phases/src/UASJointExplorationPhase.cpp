#include "uas_phases/UASJointExplorationPhase.h"
#include <cmath>


UASJointExplorationPhase::UASJointExplorationPhase()
{
    phaseName_ = "jointExploration";
}

UASState UASJointExplorationPhase::generateDesiredState(const CVImg& cvImg, const UASState& uasState)
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
    // desiredUASState.bxV_ = bodyX * cos(uasState.ipsi_  + M_PI_2) - bodyY * sin(uasState.ipsi_ + M_PI_2);
    // desiredUASState.byV_ = bodyX * sin(uasState.ipsi_  + M_PI_2) + bodyY * cos(uasState.ipsi_ + M_PI_2);
    desiredUASState.bxV_ = bodyX * cos(uasState.ipsi_) - bodyY * sin(uasState.ipsi_);
    desiredUASState.byV_ = bodyX * sin(uasState.ipsi_) + bodyY * cos(uasState.ipsi_);
    desiredUASState.bzV_ = (desiredAltitude_ - uasState.iz_) * kpZ_;
    return desiredUASState;
}