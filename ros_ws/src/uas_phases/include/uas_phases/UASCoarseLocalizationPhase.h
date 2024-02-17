#pragma once
#include "uas_phases/UASPhase.h"
#include "uas/UAS.h"
#include "uas_helpers/RGVState.h"
#include "uas_helpers/RGV.h"
#include <Eigen/Dense>


class UASCoarseLocalizationPhase : public UASPhase
{
    public:
        //fields:
        UASCoarseLocalizationPhase();
        float velocityFactor_ = 0.1;
        float tolerance_ = 0.1;

        //localization fields
        float alpha_, rcp_, dx_, dy_,sx_, sy_, d_, theta_, rGround_, xn_, yn_, xc_, yc_, zc_, centerX_, centerY_;
      
        Eigen::Vector3d r_CameraRelRGV_, r_BodyRelRGV_ ,r_DroneENURelRGV_, r_DroneESDRelRGV_;

        Eigen::Matrix3d R1_, R2_, R3_, REnu_;

        //methods:
        UASState generateDesiredState(CVImg cvImg, UASState uasState) override;
        RGVState localize(CVImg cvImg, UAS uas, RGV rgv);
};