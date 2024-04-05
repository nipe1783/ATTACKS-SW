#include "uas_scheduler/HoverMissionScheduler.h"
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

HoverMissionScheduler::HoverMissionScheduler(std::string configPath) : Scheduler("uas_hover_mission") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    psSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", qos, std::bind(&HoverMissionScheduler::callbackPS, this, std::placeholders::_1)
    );

    ssSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera2/image_raw", qos, std::bind(&HoverMissionScheduler::callbackSS, this, std::placeholders::_1)
    );

    stateSubscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, std::bind(&HoverMissionScheduler::callbackState, this, std::placeholders::_1)
    );

    attitudeSubscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", qos, std::bind(&HoverMissionScheduler::callbackAttitude, this, std::placeholders::_1)
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

    waypointIndex_ = 0;
    goalState_ = waypoints_[0];
    offboardSetpointCounter_ = 0;
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&HoverMissionScheduler::timerCallback, this));
}

void HoverMissionScheduler::timerCallback(){

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
    std::cout << std::endl;
    currentPhase_ = "exploration";
    goalState_ = explorationPhase_->generateDesiredState(rgv2CVData_, uas_.state_);
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
    std::filesystem::path configPath = currentPath  / "configurations" / "HoverMissionSchedulerConfig.yaml";
    std::cout<<configPath<<std::endl;
    std::cout << "Starting UAS Hover mission node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HoverMissionScheduler>(configPath));
	rclcpp::shutdown();
	return 0;
}