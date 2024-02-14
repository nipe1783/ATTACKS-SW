#include "uas_phases/UASCoarseLocalizationPhase.h"
#include <cmath>
#include <Eigen/Dense>
#include "uas_helpers/RGVState.h"
#include "uas_helpers/RGV.h"

UASCoarseLocalizationPhase::UASCoarseLocalizationPhase()
{
    phaseName_ = "coarse";
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
    Camera camera = Camera(640, 360, 114.592, 64.457752);;

    //Temporarily set to 0 -- Will need to change UASState to add pitch and roll

    //For sim: 1.125 radians yFOV, 2 radians xFOV -> converted to degrees
    float iphi = 0;
    float itheta = 0;

    float alpha = 0;
    if(uas.cameras_.size()>0){
        camera = uas.cameras_[0];
    }
    else{
        std::cout<< " Unable to access camera from index 0. Vector size = 0" << std::endl;
    }
    
    //Start calculations
    //DOUBLE CHECK WHETHER CVIMG WIDTH IS X PIXELS AND Y PIXELS

    float xcp = float(cvImg.width)/2;
    float ycp = float(cvImg.height)/2;

    //Distance from camera to center CF image assume alpha = 0
    float rcp = uas.state_.iz_ * (1 / cos(alpha));

    //Distance from edge to center point
    float dx = rcp * tan(camera.xFOV_/2);
    float dy = rcp * tan(camera.yFOV_/2);

    //Pixel to distance scaling factor
    float sx = xcp/dx;
    float sy = ycp/dy;

    //Distance between center point and foot
    float d = sqrt(pow((xcp-float(cvImg.centerX))/sx,2) + pow((ycp-float(cvImg.centerY))/sy,2));

    //Angle between pinhole vector and foot position vector
    float theta = atan(d/rcp);

    //Distance from camera to foot position -- UPDATE UAS STATE TO INCLUDE THETA AND PHI
    float r_ground = rcp * (1/cos(itheta));

    //Normalized foot position -- DOUBLE CHECK WHETHER CVIMG WIDTH IS X PIXELS AND Y PIXELS
    float xn = (float(cvImg.centerX)-xcp)/float(cvImg.width);
    float yn = (float(cvImg.centerY-ycp))/float(cvImg.height);

    //RGV position in camera frame
    float xc = xn * r_ground;
    float yc = yn * r_ground;
    float zc = 0;

    Eigen::MatrixXd r_cameraRelRGV(3,1);
    r_cameraRelRGV(0,0) = xc;
    r_cameraRelRGV(1,0) = yc;
    r_cameraRelRGV(2,0) = zc;
    Eigen::MatrixXd r_bodyRelRGV = r_cameraRelRGV;
    r_bodyRelRGV /= 3.281; //feet to meters  

    std::cout << "rbody x:" <<r_bodyRelRGV(0,0) << ", y:" << r_bodyRelRGV(1,0)  << ", z:" << r_bodyRelRGV(2,0) << "\n" << std::endl;

    //Rotation Matricies -- MAY NEED TO UPDATE PHI AND THETA FOR UAS - PHI = ROLL, THETA = PITCH
    Eigen::MatrixXd R1(3,3);
    R1(0,0) = 1;
    R1(0,1) = 0;
    R1(0,2) = 0;
    R1(1,0) = 0;
    R1(1,1) = cos(iphi);
    R1(1,2) = sin(iphi);
    R1(2,0) = 0;
    R1(2,1) = -sin(iphi);
    R1(2,2) = cos(iphi);
    Eigen::MatrixXd R2(3,3);
    R2(0,0) = cos(itheta);
    R2(0,1) = 0;
    R2(0,2) = -sin(itheta);
    R2(1,0) = 0;
    R2(1,1) = 1;
    R2(1,2) = 0;
    R2(2,0) = sin(itheta);
    R2(2,1) = 0;
    R2(2,2) = cos(itheta);
    Eigen::MatrixXd R3(3,3);
    R3(0,0) = cos(uas.state_.ipsi_);
    R3(0,1) = sin(uas.state_.ipsi_);
    R3(0,2) = 0;
    R3(1,0) = -sin(uas.state_.ipsi_);
    R3(1,1) = cos(uas.state_.ipsi_);
    R3(1,2) = 0;
    R3(2,0) = 0;
    R3(2,1) = 0;
    R3(2,2) = 1;

    //Added * signs in between R1, 2, 3
    Eigen::MatrixXd R_eb = R1*R2*R3;
    Eigen::MatrixXd R_be = R_eb.transpose();
    Eigen::MatrixXd r_droneESDRelRGV = R_be*r_bodyRelRGV;

    std::cout << "rdrone x:" <<r_droneESDRelRGV(0,0) << ", y:" << r_droneESDRelRGV(1,0)  << ", z:" << r_droneESDRelRGV(2,0) << "\n" << std::endl;

    Eigen::MatrixXd R4(3,3);
    R4(0,0) = 1;
    R4(0,1) = 0;
    R4(0,2) = 0;
    R4(1,0) = 0;
    R4(1,1) = -1;
    R4(1,2) = 0;
    R4(2,0) = 0;
    R4(2,1) = 0;
    R4(2,2) = -1;

    Eigen::MatrixXd r_droneENUrelRGV = R4*r_droneESDRelRGV;
    std::cout << "localize x:" <<r_droneENUrelRGV(0,0) << ", y:" << r_droneENUrelRGV(1,0)  << ", z:" << r_droneENUrelRGV(2,0) << "\n" << std::endl;

    // Assuming drone is in NED frame and not ESD frame -- transforming from NED to ENU
    /*MatrixXd r_droneENUrelRGV(3,1);
    r_droneENUrelRGV(0,0) = r_droneESDRelRGV(1,0);
    r_droneENUrelRGV(1,0) = r_droneESDRelRGV(0,0);
    r_droneENUrelRGV(2,0) = -r_droneESDRelRGV(2,0);
    */
    // Adding UAS position to get RGV position
    r_droneENUrelRGV(0,0) += uas.state_.ix_;
    r_droneENUrelRGV(1,0) += uas.state_.iy_;
    r_droneENUrelRGV(2,0) += uas.state_.iz_;
    //converting from matrix to RGV State


    RGVState rgvState = RGVState(r_droneENUrelRGV(0,0), r_droneENUrelRGV(1,0), r_droneENUrelRGV(2,0));
    rgv.state_ = rgvState;
    return rgvState;
}
