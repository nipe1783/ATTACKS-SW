#include "uas_phases/UASPhase.h"

class UASTrailingPhase : public UASPhase
{
    public:
        //fields:
        UASTrailingPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //methods:
        UASState generateDesiredState(const CVImg& cvImg, const UASState& uasState) override;

};