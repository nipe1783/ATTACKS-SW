#pragma once
#include "uas_phases/UASPhase.h"
#include "uas/UAS.h"
#include "uas_helpers/RGVState.h"
#include "uas_helpers/RGV.h"


class UASCoarseLocalizationPhase : public UASPhase
{
    public:
        //fields:
        UASCoarseLocalizationPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //methods:
        UASState generateDesiredState(CVImg cvImg, UASState uasState) override;
        RGVState localize(CVImg cvImg, UAS uas, RGV rgv);
};