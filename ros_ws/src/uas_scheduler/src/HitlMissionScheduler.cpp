#include "uas_scheduler/HitlMissionScheduler.h"
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
#include <ctime>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>

HitlMissionScheduler::HitlMissionScheduler(std::string configPath, std::string csvPath) : Scheduler("uas_hitl_mission") {
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
    .keep_last(5)
    .best_effort()
    .durability_volatile();

    myFile_.open(csvPath);
    if (!myFile_.is_open()){
        std::cout << "Unable to open CSV file" << std::endl;
    }
    myFile_ << "Timer Callback Calls (ns), UAS X, UAS Y, UAS Z, UAS Phi, UAS Theta, UAS Psi, RGV1 CV X, RGV1 CV Y, RGV1 CV Width, RGV1 CV Height, RGV2 CV X, RGV2 CV Y, RGV2 CV Width, RGV2 CV Height,";
    myFile_ << "RGV1 X (Truth), RGV1 Y (Truth), RGV1 Z (Truth), RGV2 X (Truth), RGV2 Y (Truth), RGV2 Z (Truth),";
    myFile_ << "RGV1 X (Estimate), RGV1 Y (Estimate), RGV1 Z (Estimate), RGV2 X (Estimate), RGV2 Y (Estimate), RGV2 Z (Estimate) \n";

    stateSubscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, std::bind(&HitlMissionScheduler::callbackState, this, std::placeholders::_1)
    );

    attitudeSubscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos, std::bind(&HitlMissionScheduler::callbackAttitude, this, std::placeholders::_1)
    );

    rgv1TruthSubscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/rgv1_truth/pose/uas_i_frame", qos, std::bind(&HitlMissionScheduler::callbackRGV1TruthState, this, std::placeholders::_1)
    );

    rgv2TruthSubscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/rgv2_truth/pose/uas_i_frame", qos, std::bind(&HitlMissionScheduler::callbackRGV2TruthState, this, std::placeholders::_1)
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
    YAML::Node config = YAML::LoadFile(configPath);
    rgv1_ = RGV(
        config["rgv1"]["id"].as<int>(),
        config["rgv1"]["hsvColor"]["hLow"].as<int>(),
        config["rgv1"]["hsvColor"]["hHigh"].as<int>(),
        config["rgv1"]["hsvColor"]["sLow"].as<int>(),
        config["rgv1"]["hsvColor"]["sHigh"].as<int>(),
        config["rgv1"]["hsvColor"]["vLow"].as<int>(),
        config["rgv1"]["hsvColor"]["vHigh"].as<int>()
    );
    rgv2_ = RGV(
        config["rgv2"]["id"].as<int>(),
        config["rgv2"]["hsvColor"]["hLow"].as<int>(),
        config["rgv2"]["hsvColor"]["hHigh"].as<int>(),
        config["rgv2"]["hsvColor"]["sLow"].as<int>(),
        config["rgv2"]["hsvColor"]["sHigh"].as<int>(),
        config["rgv2"]["hsvColor"]["vLow"].as<int>(),
        config["rgv2"]["hsvColor"]["vHigh"].as<int>()
    );
    camera1_ = Camera(
        config["camera1"]["id"].as<int>(),
        config["camera1"]["width"].as<int>(),
        config["camera1"]["height"].as<int>(),
        config["camera1"]["xFOV"].as<double>(),
        config["camera1"]["yFOV"].as<double>()
    );
    camera2_ = Camera(
        config["camera2"]["id"].as<int>(),
        config["camera2"]["width"].as<int>(),
        config["camera2"]["height"].as<int>(),
        config["camera2"]["xFOV"].as<double>(),
        config["camera2"]["yFOV"].as<double>()
    );
    uas_.addCamera(camera1_);
    uas_.addCamera(camera2_);
    maxHeight_ = config["maxHeight"].as<float>();
    minHeight_ = config["minHeight"].as<float>();
    stopTimeThresh_ = config["phase"]["trailing"]["stopTimeThresh"].as<float>();
    stopVelocityThresh_ = config["phase"]["trailing"]["stopVelocityThresh"].as<float>();
    coarseLocalizationTime_ = config["phase"]["coarse"]["coarseLocalizationTime"].as<float>();
    fineLocalizationTime_ = config["phase"]["fine"]["fineLocalizationTime"].as<float>();
    if (config["phase"]["exploration"]["waypoints"].IsSequence()) {
        for (const auto& wp : config["phase"]["exploration"]["waypoints"]) {
            UASState waypoint(
                wp["ix"].as<float>(),
                wp["iy"].as<float>(),
                wp["iz"].as<float>(),
                wp["ipsi"].as<float>(),
                wp["bxV"].as<float>(),
                wp["byV"].as<float>(),
                wp["bzV"].as<float>()
            );
            waypoints_.push_back(waypoint);
        }
    }

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
    trailingPhase_->desiredAltitude_ = config["phase"]["trailing"]["desiredAlt"].as<float>();
    trailingPhase_->kpZ_ = config["phase"]["kpZ"].as<float>();
    trailingPhase_->velocityFactor_ = config["phase"]["velocityFactor"].as<float>();

    coarsePhase_ = std::make_unique<UASCoarseLocalizationPhase>();
    coarsePhase_->desiredAltitude_ = config["phase"]["coarse"]["desiredAlt"].as<float>();
    coarsePhase_->kpZ_ = config["phase"]["kpZ"].as<float>();

    jointExplorationPhase_ = std::make_unique<UASJointExplorationPhase>();
    jointExplorationPhase_->desiredAltitude_ = maxHeight_;

    jointTrailingPhase_ = std::make_unique<UASJointTrailingPhase>();
    jointTrailingPhase_->desiredAltitude_ = maxHeight_;
    jointTrailingPhase_->kpZ_ = config["phase"]["kpZ"].as<float>();
    jointTrailingPhase_->velocityFactor_ = config["phase"]["velocityFactor"].as<float>();

    waypointIndex_ = 0;
    goalState_ = waypoints_[0];
    offboardSetpointCounter_ = 0;
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&HitlMissionScheduler::timerCallback, this));
}

bool HitlMissionScheduler::isUASStopped(RGV rgv) {
    if (sqrt(pow(uas_.state_.bxV_, 2) + pow(uas_.state_.byV_, 2)) > stopVelocityThresh_) {
        rgv.phaseStartTime_ = std::chrono::system_clock::now();
        return false;
    }
    else{
        if (std::chrono::system_clock::now() - rgv.phaseStartTime_  > std::chrono::seconds(2)) {
            return true;
        }
        return false;
    }
}

bool HitlMissionScheduler::isRGVCoarseLocalized(RGV rgv) {
    if (std::chrono::system_clock::now() - rgv.phaseStartTime_  > std::chrono::seconds(10)) {
        return true;
    }
    return false;
}

bool HitlMissionScheduler::isRGVJointLocalized(RGV rgv) {
    if(std::chrono::system_clock::now() - rgv.phaseStartTime_ > std::chrono::seconds(10)) {
        return true;
    }
    return false;
}

bool HitlMissionScheduler::areRGVsInFrame() {
    if (rgv1CVData_.blobs.size() > 0 && rgv2CVData_.blobs.size() > 0) {
        return true;
    }
    return false;
}

void HitlMissionScheduler::publishRGV1State(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {rgv1_.state_.ix_, rgv1_.state_.iy_, rgv1_.state_.iz_};
    rgv1StatePublisher_->publish(msg);
}

void HitlMissionScheduler::publishRGV2State(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {rgv2_.state_.ix_, rgv2_.state_.iy_, rgv2_.state_.iz_};
    rgv2StatePublisher_->publish(msg);
}

void HitlMissionScheduler::timerCallback(){

    if(!stateMsgReceived_) {
        return;
    }
    if (offboardSetpointCounter_ == 10) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        rgv1_.phaseStartTime_ = std::chrono::system_clock::now();
        arm();
    }

    myFile_ << (std::chrono::system_clock::now()).time_since_epoch().count() << ","; // System clock for latency calculations
    myFile_ << uas_.state_.ix_ << "," << uas_.state_.iy_ << "," << uas_.state_.iz_ << ","; // UAS Inertial Position (X, Y, Z)
    myFile_ << uas_.state_.iphi_ << "," << uas_.state_.itheta_ << "," << uas_.state_.ipsi_ << ","; // UAS Inertial Attitude (Phi, Theta, Psi)
    // rgv1CVData_ = rgv1BlobDetector_.detect(psFrame_);
    // rgv2CVData_ = rgv2BlobDetector_.detect(psFrame_);

    if(rgv1CVData_.blobs.size() > 0) {
        rgv1_.state_ = coarsePhase_->localize(camera1_, rgv1CVData_, uas_, rgv1_);
        publishRGV1State();
        myFile_ << rgv1CVData_.blobs[0].x  << "," << rgv1CVData_.blobs[0].y << "," << rgv1CVData_.blobs[0].width << "," << rgv1CVData_.blobs[0].height << ",";
    } else {
        myFile_ << "-1, -1, -1, -1,";
    }
    if(rgv2CVData_.blobs.size() > 0){
        rgv2_.state_ = coarsePhase_->localize(camera2_, rgv2CVData_, uas_, rgv2_);
        publishRGV2State();
        myFile_ << rgv2CVData_.blobs[0].x  << "," << rgv2CVData_.blobs[0].y << "," << rgv2CVData_.blobs[0].width << "," << rgv2CVData_.blobs[0].height << ",";
    } else {
        myFile_ << "-1, -1, -1, -1,";
    }
    myFile_ << rgv1Truth_.state_.ix_ << "," << rgv1Truth_.state_.iy_ << "," << rgv1Truth_.state_.iz_ << ","; // RGV1 Truth Inertial Position
    myFile_ << rgv2Truth_.state_.ix_ << "," << rgv2Truth_.state_.iy_ << "," << rgv2Truth_.state_.iz_ << ","; // RGV2 Truth Inertial Position
    myFile_ << rgv1_.state_.ix_ << "," << rgv1_.state_.iy_ << "," << rgv1_.state_.iz_ << ","; // RGV1 Estimate Interial Position
    myFile_ << rgv2_.state_.ix_ << "," << rgv2_.state_.iy_ << "," << rgv2_.state_.iz_ << ","; // RGV2 Estimate Inertial Position
    myFile_ << "\n";
    currentPhase_ = "exploration";
    goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
    std::cout<<currentPhase_<<std::endl;
    if(std::chrono::system_clock::now() - rgv1_.phaseStartTime_ > std::chrono::seconds(60)){
        exit(0);
    }
    publishControlMode();
    publishTrajectorySetpoint(goalState_);
    if (offboardSetpointCounter_ < 11) {
        offboardSetpointCounter_++;
    }
}


int main(int argc, char *argv[])
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::filesystem::path configPath = currentPath  / "configurations" / "HitlMissionSchedulerConfig.yaml";
    std::filesystem::path csvPath = currentPath / "hitl_data" / "data.csv";
    std::cout<<configPath<<std::endl;
    std::cout << "Starting UAS HITL mission node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HitlMissionScheduler>(configPath, csvPath));
	rclcpp::shutdown();
	return 0;
}