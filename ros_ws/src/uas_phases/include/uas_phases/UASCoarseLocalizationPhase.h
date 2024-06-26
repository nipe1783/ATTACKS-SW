#pragma once
#include "uas_phases/UASPhase.h"


class UASCoarseLocalizationPhase : public UASPhase
{
    public:
        //fields:
        UASCoarseLocalizationPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //methods:
        UASState generateDesiredState(const CVImg& cvImg, const UASState& uasState) override;
};