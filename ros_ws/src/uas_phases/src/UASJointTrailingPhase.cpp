#include "uas_phases/UASJointTrailingPhase.h"
#include <cmath>


UASJointTrailingPhase::UASJointTrailingPhase()
{
    phaseName_ = "jointTrailing";
}

UASState UASJointTrailingPhase::generateDesiredState(CVImg rgv1CVData, CVImg rgv2CVData,  UASState uasState)
{   
    UASState desiredUASState;
    desiredUASState.ix_ = 0;
    desiredUASState.iy_ = 0;
    desiredUASState.iz_ = 0;
    desiredUASState.ipsi_ = 0;
    return desiredUASState;
}