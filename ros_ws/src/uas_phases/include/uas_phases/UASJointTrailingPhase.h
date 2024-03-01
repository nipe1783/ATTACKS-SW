#include "uas_phases/UASPhase.h"

class UASJointTrailingPhase : public UASPhase
{
    public:
        //fields:
        UASJointTrailingPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //methods:
        UASState generateDesiredState(CVImg rgv1CVData, CVImg rgv2CVData, UASState uasState) override;
};