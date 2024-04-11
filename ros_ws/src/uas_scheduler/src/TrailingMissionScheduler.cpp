#include "uas_scheduler/TrailingMissionScheduler.h"
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

TrailingMissionScheduler::TrailingMissionScheduler(std::string configPath, std::string csvPath) : Scheduler("uas_trailing_mission") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    outputFile_.open(csvPath);
    if (!outputFile_.is_open()){
        std::cout << "Unable to open CSV file" << std::endl;
    }
    outputFile_ << "Timer Callback Calls (ns), UAS X, UAS Y, UAS Z, UAS Phi, UAS Theta, UAS Psi, RGV1 CV X, RGV1 CV Y, RGV1 CV Width, RGV1 CV Height, RGV2 CV X, RGV2 CV Y, RGV2 CV Width, RGV2 CV Height,";
    outputFile_ << "RGV1 X (Estimate), RGV1 Y (Estimate), RGV1 Z (Estimate), RGV2 X (Estimate), RGV2 Y (Estimate), RGV2 Z (Estimate) \n";

    psSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", qos, std::bind(&TrailingMissionScheduler::callbackPS, this, std::placeholders::_1)
    );

    stateSubscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, std::bind(&TrailingMissionScheduler::callbackState, this, std::placeholders::_1)
    );

    attitudeSubscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos, std::bind(&TrailingMissionScheduler::callbackAttitude, this, std::placeholders::_1)
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

    waypointIndex_ = 0;
    goalState_ = waypoints_[0];
    offboardSetpointCounter_ = 0;
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&TrailingMissionScheduler::timerCallback, this));
}

bool TrailingMissionScheduler::isUASStopped(RGV rgv) {
    if (sqrt(pow(uas_.state_.bxV_, 2) + pow(uas_.state_.byV_, 2)) > stopVelocityThresh_) {
        rgv.phaseStartTime_ = std::chrono::system_clock::now();
        return false;
    }
    else{
        if (std::chrono::system_clock::now() - rgv.phaseStartTime_  > std::chrono::seconds(int(stopTimeThresh_))) {
            return true;
        }
        return false;
    }
}

bool TrailingMissionScheduler::isRGVCoarseLocalized(RGV rgv) {
    if (std::chrono::system_clock::now() - rgv.phaseStartTime_  > std::chrono::seconds(int(coarseLocalizationTime_))) {
        return true;
    }
    return false;
}

bool TrailingMissionScheduler::areRGVsInFrame() {
    if (rgv1CVData_.blobs.size() > 0 && rgv2CVData_.blobs.size() > 0) {
        return true;
    }
    return false;
}

void TrailingMissionScheduler::publishRGV1State(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {rgv1_.state_.ix_, rgv1_.state_.iy_, rgv1_.state_.iz_};
    rgv1StatePublisher_->publish(msg);
}

void TrailingMissionScheduler::publishRGV2State(){
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {rgv2_.state_.ix_, rgv2_.state_.iy_, rgv2_.state_.iz_};
    rgv2StatePublisher_->publish(msg);
}

void TrailingMissionScheduler::callbackPS(const sensor_msgs::msg::Image::SharedPtr psMsg) {
    psMsgReceived_ = true;
    imageConvertPS(psMsg);
}

void TrailingMissionScheduler::imageConvertPS(const sensor_msgs::msg::Image::SharedPtr sImg)
{
    try {
        psFrame_ = cv_bridge::toCvCopy(sImg, "bgr8")->image;
        psDisplayFrame_ = psFrame_.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", sImg->encoding.c_str());
    }
}

void TrailingMissionScheduler::savePSFrame()
{
    if (!psFrame_.empty()) {
        std::string filename = "saved_image_.jpg";
        cv::imwrite(filename, psFrame_);
        RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
    }
}

void TrailingMissionScheduler::timerCallback(){

    if(!psMsgReceived_ || !stateMsgReceived_) {
        return;
    }

    if (offboardSetpointCounter_ == 10) {
        std::cout<<"ARMING UAS"<<std::endl;
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
    }

    rgv1CVData_ = rgv1BlobDetector_.detect(psFrame_);


    outputFile_ << (std::chrono::system_clock::now()).time_since_epoch().count() << ","; 
    outputFile_ << uas_.state_.ix_ << "," << uas_.state_.iy_ << "," << uas_.state_.iz_ << ",";
    outputFile_ << uas_.state_.iphi_ << "," << uas_.state_.itheta_ << "," << uas_.state_.ipsi_ << ",";
    if(rgv1CVData_.blobs.size() > 0) {
        outputFile_ << rgv1CVData_.blobs[0].x  << "," << rgv1CVData_.blobs[0].y << "," << rgv1CVData_.blobs[0].width << "," << rgv1CVData_.blobs[0].height << ",";
    } else {
        outputFile_ << "-1, -1, -1, -1,";
    }
    if(rgv2CVData_.blobs.size() > 0){
        outputFile_ << rgv2CVData_.blobs[0].x  << "," << rgv2CVData_.blobs[0].y << "," << rgv2CVData_.blobs[0].width << "," << rgv2CVData_.blobs[0].height << ",";
    } else {
        outputFile_ << "-1, -1, -1, -1,";
    }
    outputFile_ << rgv1_.state_.ix_ << "," << rgv1_.state_.iy_ << "," << rgv1_.state_.iz_ << ",";
    outputFile_ << rgv2_.state_.ix_ << "," << rgv2_.state_.iy_ << "," << rgv2_.state_.iz_ << ",";
    outputFile_ << "\n";



    if(rgv1_.currentPhase_ == "exploration" && currentPhase_ == "exploration" && rgv1CVData_.blobs.size() > 0 && uas_.state_.iz_ <= minHeight_){
        std::cout<<"TRAILING PHASE"<<std::endl;
        rgv1_.currentPhase_ = "trailing";
        currentPhase_ = "trailing";
        rgv1_.phaseStartTime_ = std::chrono::system_clock::now();
        goalState_ = trailingPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
        rgv1_.state_ = coarsePhase_->localize(camera1_, rgv1CVData_, uas_, rgv1_);
    }
    else if (rgv1_.currentPhase_ == "trailing" && currentPhase_ == "trailing" && rgv1CVData_.blobs.size() > 0){
        if (isUASStopped(rgv1_)) {
            std::cout<<"COARSE PHASE"<<std::endl;
            rgv1_.currentPhase_ = "coarse";
            currentPhase_ = "coarse";
            rgv1_.phaseStartTime_ = std::chrono::system_clock::now();
            goalState_ = coarsePhase_->generateDesiredState(rgv1CVData_, uas_.state_);
        }
        else{
            goalState_ = trailingPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
        }
    }
    else if (rgv1_.currentPhase_ == "coarse" && currentPhase_ == "coarse" && rgv1CVData_.blobs.size() > 0){
        if (isRGVCoarseLocalized(rgv1_)) {
            exit(0);
        }
        else{
            goalState_ = coarsePhase_->generateDesiredState(rgv1CVData_, uas_.state_);
            rgv1_.state_ = coarsePhase_->localize(camera1_, rgv1CVData_, uas_, rgv1_);
            publishRGV1State();
        }
    }
    else{
        currentPhase_ = "exploration";
        goalState_ = explorationPhase_->generateDesiredState(rgv1CVData_, uas_.state_);
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
    std::filesystem::path configPath = currentPath  / "configurations" / "TrailingMissionSchedulerConfig.yaml";
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    std::string fileName = "output_" + ss.str() + ".csv";
    std::filesystem::path outputPath = currentPath / ".." / "Datasets" / fileName;
    std::filesystem::create_directories(outputPath.parent_path());
    std::cout << "Config Path: " << configPath << std::endl;
    std::cout << "Output Path: " << outputPath << std::endl;
    std::cout << "Starting UAS trailing mission node..." << std::endl;
    std::cout<<configPath<<std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrailingMissionScheduler>(configPath, outputPath));
	rclcpp::shutdown();
	return 0;
}