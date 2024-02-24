#include "uas_scheduler/CompleteMissionScheduler.h"
#include "uas_scheduler/Scheduler.h"
#include "uas/UAS.h"
#include "uas_helpers/Camera.h"
#include "uas_helpers/RGV.h"
#include "uas_computer_vision/BasicBlobDetector.h"
#include "uas_computer_vision/Blob.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <iostream>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

CompleteMissionScheduler::CompleteMissionScheduler(UAS uas, RGV rgv1, RGV rgv2) : Scheduler("uas_complete_mission", uas) {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    psSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", qos, std::bind(&CompleteMissionScheduler::callbackPS, this, std::placeholders::_1)
    );

    stateSubscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, std::bind(&CompleteMissionScheduler::callbackState, this, std::placeholders::_1)
    );

    attitudeSubscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos, std::bind(&CompleteMissionScheduler::callbackAttitude, this, std::placeholders::_1)
    );

    controlModePublisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", qos
    );

    trajectorySetpointPublisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", qos
    );

    vehicleCommandPublisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", qos
    );

    rgv1StatePublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/rgv1/state", qos
    );

    rgv2StatePublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/rgv2/state", qos
    );

    rgv1_ = rgv1;
    rgv2_ = rgv2;

    // Setting blob detector parameters:
    rgv1BlobDetector_.hLow_ = rgv1_.hLow_;
    rgv1BlobDetector_.hHigh_ = rgv1_.hHigh_;
    rgv1BlobDetector_.sLow_ = rgv1_.sLow_;
    rgv1BlobDetector_.sHigh_ = rgv1_.sHigh_;
    rgv1BlobDetector_.vLow_ = rgv1_.vLow_;
    rgv1BlobDetector_.vHigh_ = rgv1_.vHigh_;
    rgv2BlobDetector_.hLow_ = rgv2_.hLow_;
    rgv2BlobDetector_.hHigh_ = rgv2_.hHigh_;
    rgv2BlobDetector_.sLow_ = rgv2_.sLow_;
    rgv2BlobDetector_.sHigh_ = rgv2_.sHigh_;
    rgv2BlobDetector_.vLow_ = rgv2_.vLow_;
    rgv2BlobDetector_.vHigh_ = rgv2_.vHigh_;

    currentPhase_ = "exploration";
    explorationPhase_ = std::make_unique<UASExplorationPhase>(waypoints_);
    trailingPhase_ = std::make_unique<UASTrailingPhase>();
    coarsePhase_ = std::make_unique<UASCoarseLocalizationPhase>();
    waypointIndex_ = 0;
    goalState_ = waypoints_[0];
    offboardSetpointCounter_ = 0;
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&CompleteMissionScheduler::timerCallback, this));
    stopVelocityThresh_ = 0.01;
}


void CompleteMissionScheduler::checkUASStopped() {
    if (uasStopped_) {
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - uasStoppedTime_).count();
        if (elapsedTime >= 2000 && currentPhase_ == "trailing") { // 2 seconds and in trailing mode
            currentPhase_ = "coarse";
            std::cout << "Phase changed to coarse after UAS stopped for 2 seconds in trailing mode.\n";
        }
    }
}

void CompleteMissionScheduler::publishRGV1State(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {rgv1_.state_.ix_, rgv1_.state_.iy_, rgv1_.state_.iz_};
    rgv1StatePublisher_->publish(msg);
}

void CompleteMissionScheduler::publishRGV2State(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {rgv2_.state_.ix_, rgv2_.state_.iy_, rgv2_.state_.iz_};
    rgv2StatePublisher_->publish(msg);
}

void CompleteMissionScheduler::timerCallback(){
    if(!psMsgReceived_ || !stateMsgReceived_) {
        return;
    }
    if (offboardSetpointCounter_ == 10) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
    }

    rgv1CVData_ = rgv1BlobDetector_.detect(psFrame_);
    rgv2CVData_ = rgv2BlobDetector_.detect(psFrame_);

    std::cout << "Phase: " << currentPhase_ << ". ";
    std::cout << "UAS State: " << uas_.state_.ix_ << ", " << uas_.state_.iy_ << ", " << uas_.state_.iz_ << ". ";
    if(rgv1CVData_.blobs.size() > 0){
        cv::Rect bounding_box = cv::Rect(rgv1CVData_.blobs[0].x, rgv1CVData_.blobs[0].y, rgv1CVData_.blobs[0].width, rgv1CVData_.blobs[0].height);
        cv::rectangle(psDisplayFrame_, bounding_box, cv::Scalar(255, 0, 0), 2);
        std::cout << "RGV1 detected. ";
    }
    if(rgv2CVData_.blobs.size() > 0){
        cv::Rect bounding_box = cv::Rect(rgv2CVData_.blobs[0].x, rgv2CVData_.blobs[0].y, rgv2CVData_.blobs[0].width, rgv2CVData_.blobs[0].height);
        cv::rectangle(psDisplayFrame_, bounding_box, cv::Scalar(0, 255, 0), 2);
        std::cout << "RGV2 detected. ";
    }
    std::cout << std::endl;

    if(rgv2CVData_.blobs.size() > 0){
        if(currentPhase_ == "exploration"){
            currentPhase_ = "trailing";
        goalState_ = trailingPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
        }
        else if(currentPhase_ == "trailing"){
            if (sqrt(pow(uas_.state_.bxV_, 2) + pow(uas_.state_.byV_, 2) + pow(uas_.state_.bzV_, 2)) < stopVelocityThresh_) { 
                if (!uasStopped_) {
                    uasStopped_ = true;
                    uasStoppedTime_ = std::chrono::steady_clock::now();
                }
            } 
            else {
                    uasStopped_ = false;
                }
            checkUASStopped();
            if(currentPhase_ == "coarse"){
                goalState_ = coarsePhase_->generateDesiredState(rgv2CVData_, uas_.state_);
            }
            else{
                goalState_ = trailingPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
            }
        }
        else if(currentPhase_ == "coarse"){
            goalState_ = coarsePhase_->generateDesiredState(rgv2CVData_, uas_.state_);
            rgv2_.state_ = coarsePhase_->localize(rgv2CVData_, uas_, rgv2_);
            publishRGV2State();
        }
    }
    else if(rgv2CVData_.blobs.size() == 0){
        if(currentPhase_ == "exploration"){
            goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
        }
        else if(currentPhase_ == "trailing"){
            currentPhase_ = "exploration";
            goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
        }
        else if(currentPhase_ == "coarse"){
            currentPhase_ = "exploration";
            goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
        }
    }
    else{
        currentPhase_ = "exploration";
        goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
    }



    cv::imshow("Primary Sensor", psDisplayFrame_);
    cv::waitKey(1);

    publishControlMode();
    publishTrajectorySetpoint(goalState_);
    if (offboardSetpointCounter_ < 11) {
        offboardSetpointCounter_++;
    }
}


int main(int argc, char *argv[])
{
	std::cout << "Starting UAS complete mission node..." << std::endl;
    RGV rgv1 = RGV(1, 0, 30, 180, 255, 180, 255); // RED
    RGV rgv2 = RGV(2, 0, 10, 0, 10, 180, 255); // WHITE
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    UAS missionUAS = UAS();
    Camera camera1 = Camera(640, 360, 2, 1.125);
    missionUAS.addCamera(camera1);
	rclcpp::spin(std::make_shared<CompleteMissionScheduler>(missionUAS, rgv1, rgv2));
	rclcpp::shutdown();
	return 0;
}