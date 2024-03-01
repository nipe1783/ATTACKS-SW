#include "uas_phases/UASPhase.h"

class UASJointExplorationPhase : public UASPhase
{
    public:
        //fields:
        UASJointExplorationPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //methods:
        UASState generateDesiredState(CVImg cvImg, UASState uasState) override;
        UASState generateDesiredState(CVImg rgv1, CVImg rgv2, UASState uasState) override;
};