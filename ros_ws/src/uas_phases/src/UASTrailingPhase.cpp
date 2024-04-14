#include "uas_phases/UASTrailingPhase.h"
#include <cmath>


UASTrailingPhase::UASTrailingPhase()
{
    phaseName_ = "Trailing";
}

UASState UASTrailingPhase::generateDesiredState(const CVImg& cvImg, const UASState& uasState)
{   
    UASState desiredUASState;
    double maxPitchAngle_ = -5 * M_PI / 180;  // Pitching forward by -5 degrees
    double maxRollAngle_ = 5 * M_PI / 180;  // Rolling right by 5 degrees

    // Get the blob coordinates
    Blob blob = cvImg.blobs[0];

    // Calculate pitch and roll angles based on blob position
    float pitch = ((blob.y - cvImg.centerY) / static_cast<float>(cvImg.height)) * maxPitchAngle_;  // maxPitchAngle_ is the maximum pitch in radians
    float roll = -((blob.x - cvImg.centerX) / static_cast<float>(cvImg.width)) * maxRollAngle_;   // maxRollAngle_ is the maximum roll in radians

    // Assuming small angles, calculate quaternion components
    desiredUASState.q0_ = cos(pitch / 2) * cos(roll / 2);
    desiredUASState.q1_ = sin(roll / 2) * cos(pitch / 2);
    desiredUASState.q2_ = cos(roll / 2) * sin(pitch / 2);
    desiredUASState.q3_ = sin(roll / 2) * sin(pitch / 2);

    // Set thrust to maintain altitude while tilting
    desiredUASState.thrustX_ = 0;
    desiredUASState.thrustY_ = 0;
    desiredUASState.thrustZ_ = -0.455; // Example thrust value, adjust as necessary

    return desiredUASState;
}