#include "uas_phases/UASTrailingPhase.h"
#include <cmath>


UASTrailingPhase::UASTrailingPhase()
{
    phaseName_ = "Trailing";
}

UASState UASTrailingPhase::generateDesiredState(const CVImg& cvImg, const UASState& uasState)
{   
    UASState desiredUASState;
    desiredUASState.bxV_ = 0.0f;
    desiredUASState.byV_ = 0.0f;
    desiredUASState.bzV_ = 0.0f;
    float bodyX = 0.0f;
    float bodyY = 0.0f;

    // std::cout<<"Blob x: "<<cvImg.blobs[0].x<< " Blob y: "<<cvImg.blobs[0].y<<std::endl;
    // Blob blob = cvImg.blobs[0];
    // if (std::abs(blob.x - cvImg.centerX) / static_cast<float>(cvImg.width) > tolerance_) {
    //     bodyX = (blob.x - cvImg.centerX) / static_cast<float>(cvImg.width) * velocityFactor_;
    // }

    // if (std::abs(blob.y - cvImg.centerY) / static_cast<float>(cvImg.height) > tolerance_) {
    //     bodyY = (blob.y - cvImg.centerY) / static_cast<float>(cvImg.height) * velocityFactor_;
    // }
    // desiredUASState.bxV_ = bodyX * cos(uasState.ipsi_  + M_PI_2) - bodyY * sin(uasState.ipsi_ + M_PI_2);
    // desiredUASState.byV_ = bodyX * sin(uasState.ipsi_  + M_PI_2) + bodyY * cos(uasState.ipsi_ + M_PI_2);
    // desiredUASState.bzV_ = (desiredAltitude_ - uasState.iz_) * kpZ_;
    desiredUASState.bxV_ = 1.0f;
    desiredUASState.byV_ = 0.0f;
    desiredUASState.bzV_ = 0.0f;
    std::cout<<"Desired bxV: "<<desiredUASState.bxV_<<" Desired byV: "<<desiredUASState.byV_<<" Desired bzV: "<<desiredUASState.bzV_<<std::endl;

    return desiredUASState;
}