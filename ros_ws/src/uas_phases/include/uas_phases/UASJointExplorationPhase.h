#include "uas_phases/UASPhase.h"

class UASJointExplorationPhase : public UASPhase
{
    public:
        //fields:
        UASJointExplorationPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //methods:
        UASState generateDesiredState(const CVImg& rgv1CVData, const UASState& uasState) override;
};