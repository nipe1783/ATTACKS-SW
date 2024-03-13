#pragma once
#include "uas_phases/UASPhase.h"


class UASFineLocalizationPhase : public UASPhase
{
    public:
        //fields:
        UASFineLocalizationPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //methods:
        UASState generateDesiredState(const CVImg& cvImg, const UASState& uasState) override;
};