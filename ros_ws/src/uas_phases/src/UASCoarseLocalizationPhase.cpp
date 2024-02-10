#include "uas_phases/UASCoarseLocalizationPhase.h"
#include <cmath>
#include "eigen-3.4.0/Eigen/Dense"
using Eigen::MatrixXd;

UASCoarseLocalizationPhase::UASCoarseLocalizationPhase()
{
    phaseName_ = "Coarse Localization";
}

UASState UASTrailingPhase::generateDesiredState(CVImg cvImg, UASState uasState)
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
    //Temporarily set to 0 -- Will need to change UASState to add pitch and roll
    float iphi = 0;
    float itheta = 0;

    float alpha = 0;

    Camera camera = uas.cameras_[1];
    //Startr calculations

    //Distance from camera to center CF image assume alpha = 0
    float rcp = uas.iz_ * (1 / cos(alpha));

    //Distance from edge to center point
    // MAKE SURE TO UPDATE CAMERA DO WE HAVE FOV X AND FOV Y 
    float dx = rcp * tan(camera.FOV_/2);
    float dy = rcp * tan(camera.FOV_/2);

    

    return null;
}
// #include
// #include
// #include
// #include
// #include
// #include "eigen-3.4.0/Eigen/Dense"
// using Eigen::MatrixXd;
// /struct Vehicle {
// float x;
// float y;
// float z;
// float roll;
// float pitch;
// float yaw;
// };/
// MatrixXd Localization(float x_rgv, float y_rgv, float x,float y, float z, float roll, float pitch, float yaw){
// // camera parameters
// float FOVx = 13.0736;
// float FOVy = 7.3539;
// int x_pixels = 3840;
// int y_pixlels = 2160;
// float alpha = 0;
// float x_uas = x;
// float y_uas = y;
// float z_uas = z;
// float roll_uas = roll;
// float pitch_uas = pitch;
// float yaw_uas = yaw;
// //Image center point
// float xcp = x_pixels/2;
// float ycp = y_pixlels/2;
// //Distance from camera to center CF image assume alpha = 0
// float rcp = z_uas * (1 / cos(alpha));
// //Distance from edge to center point
// float dx = rcp * tan(FOVx/2);
// float dy = rcp * tan(FOVy/2);
// //Pixel to distance scaling factor
// float sx = xcp/dx;
// float sy = ycp/dy;
// //Distance between center point and foot
// float d = sqrt(pow((xcp-x_rgv)/sx,2) + pow((ycp-y_rgv)/sy,2));
// //Angle between pinhole vector and foot position vector
// float theta = atan(d/rcp);
// //Distance from camera to foot position
// float r_ground = rcp * (1/cos(theta));
// //Normalized foot position
// float xn = (x_rgv-xcp)/x_pixels;
// float yn = (y_rgv-ycp)/y_pixlels;
// //RGV position in camera frame
// float xc = xn * r_ground;
// float yc = yn * r_ground;
// float zc = 0;
// MatrixXd r_cameraRelRGV(3,1);
// r_cameraRelRGV(0,0) = xc;
// r_cameraRelRGV(1,0) = yc;
// r_cameraRelRGV(2,0) = zc;
// MatrixXd r_bodyRelRGV = r_cameraRelRGV;
// r_bodyRelRGV /= 3.281; //feet to meters
// //Rotation Matricies
// MatrixXd R1(3,3);
// R1(0,0) = 1;
// R1(0,1) = 0;
// R1(0,2) = 0;
// R1(1,0) = 0;
// R1(1,1) = cos(roll_uas);
// R1(1,2) = sin(roll_uas);
// R1(2,0) = 0;
// R1(2,1) = -sin(roll_uas);
// R1(2,2) = cos(roll_uas);
// MatrixXd R2(3,3);
// R2(0,0) = cos(pitch_uas);
// R2(0,1) = 0;
// R2(0,2) = -sin(pitch_uas);
// R2(1,0) = 0;
// R2(1,1) = 1;
// R2(1,2) = 0;
// R2(2,0) = sin(pitch_uas);
// R2(2,1) = 0;
// R2(2,2) = cos(pitch_uas);
// MatrixXd R3(3,3);
// R3(0,0) = cos(yaw_uas);
// R3(0,1) = sin(yaw_uas);
// R3(0,2) = 0;
// R3(1,0) = -sin(yaw_uas);
// R3(1,1) = cos(yaw_uas);
// R3(1,2) = 0;
// R3(2,0) = 0;
// R3(2,1) = 0;
// R3(2,2) = 1;
// MatrixXd R_eb = R1R2R3;
// MatrixXd R_be = R_eb.transpose();
// MatrixXd r_droneESDRelRGV = R_ber_bodyRelRGV;
// MatrixXd R4(3,3);
// R4(0,0) = 1;
// R4(0,1) = 0;
// R4(0,2) = 0;
// R4(1,0) = 0;
// R4(1,1) = -1;
// R4(1,2) = 0;
// R4(2,0) = 0;
// R4(2,1) = 0;
// R4(2,2) = -1;
// MatrixXd r_droneENUrelRGV = R4r_droneESDRelRGV;
// // Assuming drone is in NED frame and not ESD frame -- transforming from NED to ENU
// /*MatrixXd r_droneENUrelRGV(3,1);
// r_droneENUrelRGV(0,0) = r_droneESDRelRGV(1,0);
// r_droneENUrelRGV(1,0) = r_droneESDRelRGV(0,0);
// r_droneENUrelRGV(2,0) = -r_droneESDRelRGV(2,0);
// */
// // Adding UAS position to get RGV position
// /r_droneENUrelRGV(0,0) += x_uas;
// r_droneENUrelRGV(1,0) += y_uas;
// r_droneENUrelRGV(2,0) += z_uas;/
// return r_droneENUrelRGV;
// }
// /int main()
// {
// Vehicle UAS = {1,1,1,0,0,0};
// MatrixXd solution = Localization(5, 10, UAS, 50, 50, 0, 1080, 2900);
// std::cout << solution <<std::endl;
// int rows = 6;
// int cols = 10;
// Eigen::MatrixXd matrix(rows, cols);
// std::ifstream file("DroneStates.txt");
// std::string line;
// int row = 0;
// int col = 0;
// while (std::getline(file, line) && row < rows) {
// std::stringstream lineStream(line);
// std::string cell;
// col = 0;
// while (std::getline(lineStream, cell, ',') && col < cols) {
// matrix(row, col) = std::stod(cell);
// col++;
// }
// row++;
// }
// std::cout << " " << std::endl;
// std::cout << matrix.row(0).col(0) << std::endl;
// return 0;
// }/