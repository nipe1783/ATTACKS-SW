#pragma once
#include "uas_phases/UASPhase.h"

class UASCoarseLocalizationPhase : public UASPhase
{
    public:
        //fields:
        UASTrailingPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //methods:
        UASState generateDesiredState(CVImg cvImg, UAS uasState) override;
        RGVState localize(CVImg cvImg, UAS uas);
};