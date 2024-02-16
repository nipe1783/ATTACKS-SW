#include "uas_phases/UASCoarseLocalizationPhase.h"
#include <cmath>
#include <Eigen/Dense>
#include "uas_helpers/RGVState.h"
#include "uas_helpers/RGV.h"

UASCoarseLocalizationPhase::UASCoarseLocalizationPhase()
{
    phaseName_ = "coarse";
    alpha_ = 0;
    REnu_ << 1, 0, 0,
             0, -1, 0,
             0, 0, -1;
}

UASState UASCoarseLocalizationPhase::generateDesiredState(CVImg cvImg, UASState uasState)
{   
    UASState desiredUASState;
    desiredUASState.bxV_ = 0.0f;
    desiredUASState.byV_ = 0.0f;
    desiredUASState.bzV_ = 0.0f;
    float bodyX = 0.0f;
    float bodyY = 0.0f;

    Blob blob = cvImg.blobs[0];
    if (std::abs(blob.x - cvImg.centerX) / static_cast<float>(cvImg.width) > tolerance_) {
        bodyX = (blob.x - cvImg.centerX) / static_cast<float>(cvImg.width) * 3;
    }

    if (std::abs(blob.y - cvImg.centerY) / static_cast<float>(cvImg.height) > tolerance_) {
        bodyY = (blob.y - cvImg.centerY) / static_cast<float>(cvImg.height) * 3;
    }

    desiredUASState.bxV_ = bodyX * cos(uasState.ipsi_  + M_PI_2) - bodyY * sin(uasState.ipsi_ + M_PI_2);
    desiredUASState.byV_ = bodyX * sin(uasState.ipsi_  + M_PI_2) + bodyY * cos(uasState.ipsi_ + M_PI_2);
    return desiredUASState;
}

RGVState UASCoarseLocalizationPhase::localize(CVImg cvImg, UAS uas, RGV rgv)
{
    //Initialize values
    Camera camera = Camera(640, 360, 2, 1.125);
    centerX_ = float(cvImg.centerX);
    centerY_ = float(cvImg.centerY);


    float iphi = 0;
    float itheta = 0;    
    if(uas.cameras_.size()>0){
        camera = uas.cameras_[0];
    }
    else{
        std::cout<< " Unable to access camera from index 0. Vector size = 0" << std::endl;
    }    

    //Start calculations
    //DOUBLE CHECK WHETHER CVIMG WIDTH IS X PIXELS AND Y PIXELS    //Distance from camera to center CF image assume alpha = 0
    rcp_ = uas.state_.iz_ * (1.0 / cos(alpha_));

    //Distance from edge to center point
    dx_ = rcp_ * tan(camera.xFOV_/2.0);
    dy_ = rcp_ * tan(camera.yFOV_/2.0);
    //Pixel to distance scaling factor
    sx_ = centerX_/dx_;
    sy_ = centerY_/dy_;
    //Distance between center point and foot
    d_ = sqrt(pow((centerX_-float(cvImg.blobs[0].x))/sx_,2) + pow((centerY_-float(cvImg.blobs[0].y))/sy_,2));
    //Angle between pinhole vector and foot position vector
    theta_ = atan(d_/rcp_);
    //Distance from camera to foot position -- UPDATE UAS STATE TO INCLUDE THETA AND PHI
    rGround_ = rcp_ * (1.0/cos(theta_));
    xn_ = (float(cvImg.blobs[0].x)-centerX_)/float(cvImg.width);
    yn_ = (float(cvImg.blobs[0].y)-centerY_)/float(cvImg.height);
    //RGV position in camera frame
    xc_ = xn_ * rGround_;
    yc_ = yn_ * rGround_;
    zc_ = uas.state_.iz_;

    r_CameraRelRGV_ << xc_, yc_, zc_;

    r_BodyRelRGV_ = r_CameraRelRGV_;
    
    //Rotation Matricies -- MAY NEED TO UPDATE PHI AND THETA FOR UAS - PHI = ROLL, THETA = PITCH
    R1_ << 1, 0, 0,
        0, cos(iphi), sin(iphi),
        0, -sin(iphi), cos(iphi);
   
    R2_ << cos(itheta), 0, -sin(itheta),
        0, 1, 0,
        sin(itheta), 0, cos(itheta);
    R3_ << cos(uas.state_.ipsi_), sin(uas.state_.ipsi_), 0,
        -sin(uas.state_.ipsi_), cos(uas.state_.ipsi_), 0,
        0, 0, 1;
    
    r_DroneESDRelRGV_ = ((R1_*R2_*R3_)*r_BodyRelRGV_);
    r_DroneESDRelRGV_(2) = uas.state_.iz_; // Flat Earth Assumption
    r_DroneENURelRGV_ = REnu_ * r_DroneESDRelRGV_;
     
    // Assuming drone is in NED frame and not ESD frame -- transforming from NED to ENU
    // Eigen::MatrixXd r_droneENUrelRGV(3,1);
    // r_droneENUrelRGV(0,0) = r_droneESDRelRGV(1,0);
    // r_droneENUrelRGV(1,0) = r_droneESDRelRGV(0,0);
    // r_droneENUrelRGV(2,0) = -r_droneESDRelRGV(2,0);

    // Adding UAS position to get RGV position
    r_DroneENURelRGV_(0) += uas.state_.ix_;
    r_DroneENURelRGV_(1) += uas.state_.iy_;
    r_DroneENURelRGV_(2) += uas.state_.iz_;

    //converting from matrix to RGV State    
    RGVState rgvState = RGVState(r_DroneENURelRGV_(0), r_DroneENURelRGV_(1), r_DroneENURelRGV_(2));
    rgv.state_ = rgvState;
    return rgvState;
}