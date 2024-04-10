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
#include <ctime>
#include <yaml-cpp/yaml.h>
#include <filesystem>

CompleteMissionScheduler::CompleteMissionScheduler(std::string configPath) : Scheduler("uas_complete_mission") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    psSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", qos, std::bind(&CompleteMissionScheduler::callbackPS, this, std::placeholders::_1)
    );

    ssSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera2/image_raw", qos, std::bind(&CompleteMissionScheduler::callbackSS, this, std::placeholders::_1)
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
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&CompleteMissionScheduler::timerCallback, this));
}

bool CompleteMissionScheduler::isUASStopped(RGV rgv) {
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

bool CompleteMissionScheduler::isRGVCoarseLocalized(RGV rgv) {
    if (std::chrono::system_clock::now() - rgv.phaseStartTime_  > std::chrono::seconds(10)) {
        return true;
    }
    return false;
}

bool CompleteMissionScheduler::areRGVsInFrame() {
    if (rgv1CVData_.blobs.size() > 0 && rgv2CVData_.blobs.size() > 0) {
        return true;
    }
    return false;
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

void CompleteMissionScheduler::callbackPS(const sensor_msgs::msg::Image::SharedPtr psMsg) {
    psMsgReceived_ = true;
    imageConvertPS(psMsg);
}

void CompleteMissionScheduler::callbackSS(const sensor_msgs::msg::Image::SharedPtr ssMsg) {
    ssMsgReceived_ = true;
    imageConvertSS(ssMsg);
}

void CompleteMissionScheduler::imageConvertPS(const sensor_msgs::msg::Image::SharedPtr sImg)
{
    try {
        psFrame_ = cv_bridge::toCvCopy(sImg, "bgr8")->image;
        psDisplayFrame_ = psFrame_.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", sImg->encoding.c_str());
    }
}

void CompleteMissionScheduler::imageConvertSS(const sensor_msgs::msg::Image::SharedPtr sImg)
{
    try {
        ssFrame_ = cv_bridge::toCvCopy(sImg, "bgr8")->image;
        ssDisplayFrame_ = ssFrame_.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", sImg->encoding.c_str());
    }
}

void CompleteMissionScheduler::savePSFrame()
{
    if (!psFrame_.empty()) {
        std::string filename = "saved_image_.jpg";
        cv::imwrite(filename, psFrame_);
        RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
    }
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
    std::cout<< "RGV 1 Phase: " << rgv1_.currentPhase_ << ". ";
    std::cout<< "RGV 2 Phase: " << rgv2_.currentPhase_ << ". ";
    // std::cout<< "UAS State: " << uas_.state_.ix_ << ", " << uas_.state_.iy_ << ", " << uas_.state_.iz_ << ". ";
    // if(rgv1CVData_.blobs.size() > 0){
    //     cv::Rect bounding_box = cv::Rect(rgv1CVData_.blobs[0].x, rgv1CVData_.blobs[0].y, rgv1CVData_.blobs[0].width, rgv1CVData_.blobs[0].height);
    //     cv::rectangle(psDisplayFrame_, bounding_box, cv::Scalar(255, 0, 0), 2);
    // }
    // if(rgv2CVData_.blobs.size() > 0){
    //     cv::Rect bounding_box = cv::Rect(rgv2CVData_.blobs[0].x, rgv2CVData_.blobs[0].y, rgv2CVData_.blobs[0].width, rgv2CVData_.blobs[0].height);
    //     cv::rectangle(psDisplayFrame_, bounding_box, cv::Scalar(0, 255, 0), 2);
    // }
    std::cout << std::endl;

    if(rgv1_.currentPhase_ == "exploration" && currentPhase_ == "exploration" && rgv1CVData_.blobs.size() > 0 && uas_.state_.iz_ <= minHeight_){
        rgv1_.currentPhase_ = "trailing";
        currentPhase_ = "trailing";
        rgv1_.phaseStartTime_ = std::chrono::system_clock::now();
        goalState_ = trailingPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
    }
    else if (rgv2_.currentPhase_ == "exploration" && currentPhase_ == "exploration" && rgv2CVData_.blobs.size() > 0 && uas_.state_.iz_ <= minHeight_){
        rgv2_.currentPhase_ = "trailing";
        currentPhase_ = "trailing";
        rgv2_.phaseStartTime_ = std::chrono::system_clock::now();
        goalState_ = trailingPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
    }
    else if (rgv1_.currentPhase_ == "trailing" && currentPhase_ == "trailing" && rgv1CVData_.blobs.size() > 0){
        if (isUASStopped(rgv1_)) {
            rgv1_.currentPhase_ = "coarse";
            currentPhase_ = "coarse";
            rgv1_.phaseStartTime_ = std::chrono::system_clock::now();
            goalState_ = coarsePhase_->generateDesiredState(rgv1CVData_, uas_.state_);
        }
        else{
            goalState_ = trailingPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
        }
    }
    else if (rgv2_.currentPhase_ == "trailing" && currentPhase_ == "trailing" && rgv2CVData_.blobs.size() > 0){
        if (isUASStopped(rgv2_)) {
            rgv2_.currentPhase_ = "coarse";
            currentPhase_ = "coarse";
            rgv2_.phaseStartTime_ = std::chrono::system_clock::now();
            goalState_ = coarsePhase_->generateDesiredState(rgv2CVData_, uas_.state_);
        }
        else{
            goalState_ = trailingPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
        }
    }
    else if (rgv1_.currentPhase_ == "coarse" && currentPhase_ == "coarse" && rgv1CVData_.blobs.size() > 0){
        if (isRGVCoarseLocalized(rgv1_)) {
            rgv1_.currentPhase_ = "jointExploration";
            currentPhase_ = "exploration";
            goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
        }
        else{
            goalState_ = coarsePhase_->generateDesiredState(rgv1CVData_, uas_.state_);
            rgv1_.state_ = coarsePhase_->localize(camera1_, rgv1CVData_, uas_, rgv1_);
            publishRGV1State();
        }
    }
    else if (rgv2_.currentPhase_ == "coarse" && currentPhase_ == "coarse" && rgv2CVData_.blobs.size() > 0){
        if (isRGVCoarseLocalized(rgv2_)) {
            rgv2_.currentPhase_ = "jointExploration";
            currentPhase_ = "exploration";
            goalState_ = explorationPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
        }
        else{
            goalState_ = coarsePhase_->generateDesiredState(rgv2CVData_, uas_.state_);
            rgv2_.state_ = coarsePhase_->localize(camera1_, rgv2CVData_, uas_, rgv2_);
            publishRGV2State();
        }
    }
    else if (rgv1_.currentPhase_ == "jointExploration" && rgv2_.currentPhase_ == "jointExploration" && currentPhase_ == "exploration"){
        currentPhase_ = "jointExploration";
        goalState_ = jointExplorationPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
    }
    else if (rgv1_.currentPhase_ == "jointExploration" && currentPhase_ == "jointExploration" && rgv1CVData_.blobs.size() > 0){
       if (areRGVsInFrame()) {
            rgv1_.currentPhase_ = "jointTrailing";
            rgv2_.currentPhase_ = "jointTrailing";
            currentPhase_ = "jointTrailing";
            rgv1_.phaseStartTime_ = std::chrono::system_clock::now();
            rgv2_.phaseStartTime_ = std::chrono::system_clock::now();
            goalState_ = jointTrailingPhase_->generateDesiredState(rgv1CVData_, rgv2CVData_, uas_.state_);
        }
        else{
            goalState_ = jointExplorationPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
        }
    }
    else if (rgv2_.currentPhase_ == "jointExploration" && currentPhase_ == "jointExploration" && rgv2CVData_.blobs.size() > 0){
        if (areRGVsInFrame()) {
            rgv1_.currentPhase_ = "jointTrailing";
            rgv2_.currentPhase_ = "jointTrailing";
            currentPhase_ = "jointTrailing";
            rgv1_.phaseStartTime_ = std::chrono::system_clock::now();
            rgv2_.phaseStartTime_ = std::chrono::system_clock::now();
            goalState_ = jointTrailingPhase_->generateDesiredState(rgv1CVData_, rgv2CVData_, uas_.state_);
        }
        else{
            goalState_ = jointExplorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
        }
    }
    else if (rgv1_.currentPhase_ == "jointTrailing" && rgv2_.currentPhase_ == "jointTrailing" && currentPhase_ == "jointTrailing"  && rgv1CVData_.blobs.size() > 0 && rgv2CVData_.blobs.size() > 0){
        rgv1_.state_ = jointTrailingPhase_->localize(camera1_ ,rgv1CVData_, uas_, rgv1_);
        publishRGV1State();
        rgv2_.state_ = jointTrailingPhase_->localize(camera1_, rgv2CVData_, uas_, rgv2_);
        publishRGV2State();
        goalState_ = jointTrailingPhase_->generateDesiredState(rgv1CVData_, rgv2CVData_, uas_.state_);
    }
    else if (rgv1_.currentPhase_ == "jointTrailing" && currentPhase_ == "jointTrailing" && rgv1CVData_.blobs.size() == 0){
        rgv1_.currentPhase_ = "jointExploration";
        rgv2_.currentPhase_ = "jointExploration";
        currentPhase_ = "jointExploration";
        goalState_ = jointExplorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
    }
    else if (rgv2_.currentPhase_ == "jointTrailing" && currentPhase_ == "jointTrailing" && rgv2CVData_.blobs.size() == 0){
        rgv2_.currentPhase_ = "jointExploration";
        rgv1_.currentPhase_ = "jointExploration";
        currentPhase_ = "jointExploration";
        goalState_ = jointExplorationPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
    }
    else if (rgv1_.currentPhase_ == "trailing" && currentPhase_ == "trailing" && rgv1CVData_.blobs.size() == 0){
        rgv1_.currentPhase_ = "exploration";
        currentPhase_ = "exploration";
        goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
    }
    else if (rgv2_.currentPhase_ == "trailing" && currentPhase_ == "trailing" && rgv2CVData_.blobs.size() == 0){
        rgv2_.currentPhase_ = "exploration";
        currentPhase_ = "exploration";
        goalState_ = explorationPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
    }
    else if (rgv1_.currentPhase_ == "coarse" && currentPhase_ == "coarse" && rgv1CVData_.blobs.size() == 0){
        rgv1_.currentPhase_ = "exploration";
        currentPhase_ = "exploration";
        goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
    }
    else if (rgv2_.currentPhase_ == "coarse" && currentPhase_ == "coarse" && rgv2CVData_.blobs.size() == 0){
        rgv2_.currentPhase_ = "exploration";
        currentPhase_ = "exploration";
        goalState_ = explorationPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
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
    std::filesystem::path currentPath = std::filesystem::current_path();
    std::filesystem::path configPath = currentPath  / "configurations" / "CompleteMissionSchedulerConfig.yaml";
    std::cout<<configPath<<std::endl;
    std::cout << "Starting UAS complete mission node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CompleteMissionScheduler>(configPath));
	rclcpp::shutdown();
	return 0;
}