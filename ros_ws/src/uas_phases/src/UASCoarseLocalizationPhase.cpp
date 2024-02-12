#include "uas_phases/UASCoarseLocalizationPhase.h"
#include <cmath>
#include "eigen-3.4.0/Eigen/Dense"
using Eigen::MatrixXd;

UASCoarseLocalizationPhase::UASCoarseLocalizationPhase()
{
    phaseName_ = "Coarse Localization";
}

UASState UASTrailingPhase::generateDesiredState(CVImg cvImg, UASState uasState)
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
    return desiredUASState;
}

RGVState UASCoarseLocalizationPhase::localize(CVImg cvImg, UAS uas, RGV rgv)
{
    //Initialize values
    //Temporarily set to 0 -- Will need to change UASState to add pitch and roll
    float iphi = 0;
    float itheta = 0;

    float alpha = 0;

    Camera camera = uas.cameras_[1];
    //Startr calculations

    //Distance from camera to center CF image assume alpha = 0
    float rcp = uas.iz_ * (1 / cos(alpha));

    //Distance from edge to center point
    // MAKE SURE TO UPDATE CAMERA DO WE HAVE FOV X AND FOV Y 
    float dx = rcp * tan(camera.FOV_/2);
    float dy = rcp * tan(camera.FOV_/2);

    

    return null;
}
