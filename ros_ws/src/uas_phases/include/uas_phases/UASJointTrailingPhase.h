#include "uas_phases/UASPhase.h"

class UASJointTrailingPhase : public UASPhase
{
    public:
        //fields:
        UASJointTrailingPhase();
        float velocityFactor_ = 3;
        float tolerance_ = 0.3;

        //methods:
        UASState generateDesiredState(const CVImg& rgv1CVData, const CVImg& rgv2CVData, const UASState& uasState) override;
        bool isNearFrameEdge(const Blob &blob,const CVImg &cvImg);
};