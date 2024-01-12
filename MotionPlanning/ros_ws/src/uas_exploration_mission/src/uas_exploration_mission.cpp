#include "uas_exploration_mission.h"
#include "computer_vision/BasicBlobDetector.h"
#include "computer_vision/Blob.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <iostream>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

UASExplorationMission::UASExplorationMission() : UASMission("uas_exploration_mission") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    psSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", qos, std::bind(&UASExplorationMission::callbackPS, this, std::placeholders::_1)
    );

    stateSubscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, std::bind(&UASExplorationMission::callbackState, this, std::placeholders::_1)
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

    currentPhase_ = "exploration";
    explorationPhase_ = std::make_unique<UASExplorationPhase>(waypoints_);
    waypointIndex_ = 0;
    goalState_ = waypoints_[0];
    offboardSetpointCounter_ = 0;
    
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&UASExplorationMission::timerCallback, this));
}

void UASExplorationMission::timerCallback(){
    if(!psMsgReceived_ || !stateMsgReceived_) {
        return;
    }
    if (offboardSetpointCounter_ == 10) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        arm();
    }
   
    detectedBlobs_ = blobDetector_.detect(psFrame_);
    cv::imshow("Primary Sensor", psFrame_);
    cv::waitKey(1);
    std::cout << "UAS position: " << uasState_.x << ", " << uasState_.y << ", " << uasState_.z << ". ";
    std::cout << "Number of blobs: " << detectedBlobs_.size() << std::endl;
    
    if(currentPhase_ == "exploration"){
        goalState_ = explorationPhase_->generateDesiredState(detectedBlobs_, uasState_);
    }
    
    publishControlMode();
    publishTrajectorySetpoint(goalState_);
    if (offboardSetpointCounter_ < 11) {
        offboardSetpointCounter_++;
    }
}


int main(int argc, char *argv[])
{
	std::cout << "Starting UAS exploration mission node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UASExplorationMission>());
	rclcpp::shutdown();
	return 0;
}