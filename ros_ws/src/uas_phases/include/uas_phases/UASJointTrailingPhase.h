#include "uas_phases/UASPhase.h"

class UASJointTrailingPhase : public UASPhase
{
    public:
        //fields:
        UASJointTrailingPhase();
        float velocityFactor_ = 3;
        float tolerance_ = 0.3;

        //methods:
        UASState generateDesiredState(RGV rgv1, RGV rgv2, CVImg rgv1CVData, CVImg rgv2CVData, UASState uasState) override;
        bool isNearFrameEdge(const Blob &blob,const CVImg &cvImg);
};