#include "uas_phases/UASJointTrailingPhase.h"
#include "uas_helpers/RGV.h"
#include <cmath>


UASJointTrailingPhase::UASJointTrailingPhase()
{
    phaseName_ = "jointTrailing";
}

bool UASJointTrailingPhase::isNearFrameEdge(const Blob &blob, const CVImg &cvImg)
{   
    if( blob.x < cvImg.width * tolerance_ || blob.x > cvImg.width * (1 - tolerance_) || blob.y < cvImg.height * tolerance_ || blob.y > cvImg.height * (1 - tolerance_)){
        return true;
    }
    return false;
}

UASState UASJointTrailingPhase::generateDesiredState(const CVImg& rgv1CVData, const CVImg& rgv2CVData, const UASState& uasState)
{   
    UASState desiredUASState;
    desiredUASState.bxV_ = 0.0f;
    desiredUASState.byV_ = 0.0f;
    desiredUASState.bzV_ = 0.0f;
    float bodyX = 0.0f;
    float bodyY = 0.0f;
    Blob rgv1Blob = rgv1CVData.blobs[0];
    Blob rgv2Blob = rgv2CVData.blobs[0];

    if(isNearFrameEdge(rgv1Blob, rgv1CVData)){
        bodyX = (rgv1Blob.x - rgv1CVData.centerX) / static_cast<float>(rgv1CVData.width) * velocityFactor_;
        bodyY = (rgv1Blob.y - rgv1CVData.centerY) / static_cast<float>(rgv1CVData.height) * velocityFactor_;
    }
    else if(isNearFrameEdge(rgv2Blob, rgv2CVData)){
        bodyX = (rgv2Blob.x - rgv2CVData.centerX) / static_cast<float>(rgv2CVData.width) * velocityFactor_;
        bodyY = (rgv2Blob.y - rgv2CVData.centerY) / static_cast<float>(rgv2CVData.height) * velocityFactor_;
    }
    desiredUASState.bxV_ = bodyX * cos(uasState.ipsi_  + M_PI_2) - bodyY * sin(uasState.ipsi_ + M_PI_2);
    desiredUASState.byV_ = bodyX * sin(uasState.ipsi_  + M_PI_2) + bodyY * cos(uasState.ipsi_ + M_PI_2);
    desiredUASState.bzV_ = (desiredAltitude_ - uasState.iz_) * kpZ_;
    return desiredUASState;
}