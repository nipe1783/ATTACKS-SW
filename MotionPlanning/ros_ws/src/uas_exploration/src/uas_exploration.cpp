#include "uas_exploration.h"
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

UASExploration::UASExploration() : UASPhaseNode("uas_exploration_node") {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    psSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", qos, std::bind(&UASExploration::callbackPS, this, std::placeholders::_1)
    );

    stateSubscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, std::bind(&UASExploration::callbackState, this, std::placeholders::_1)
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

    waypointIndex_ = 0;
    goalPosition_ = waypoints_[0];
    offboardSetpointCounter_ = 0;
    auto timerCallback = [this]() -> void {
        if (offboardSetpointCounter_ == 10) {
            publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
        }
        action();
        if (offboardSetpointCounter_ < 11) {
            offboardSetpointCounter_++;
        }
    };
    timer_ = create_wall_timer(std::chrono::milliseconds(50), timerCallback);
}

void UASExploration::action() {

    if(!psMsgReceived_ || !stateMsgReceived_) {
        return;
    }
    cv::imshow("Primary Sensor", psFrame_);
    cv::waitKey(1);
    std::vector<Blob> blobs = blobDetector_.detect(psFrame_);
    std::cout << "UAS position: " << uasPosition_.x << ", " << uasPosition_.y << ", " << uasPosition_.z << ". ";
    std::cout << "Number of blobs: " << blobs.size() << std::endl;
    if(distance(uasPosition_, goalPosition_) < 0.5) {
        goalPosition_ = waypoints_[waypointIndex_];
        waypointIndex_++;
        if(waypointIndex_ == waypoints_.size()) {
            waypointIndex_ = 0;
        }
    }
    publishControlMode();
    publishTrajectorySetpoint(goalPosition_);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting UAS exploration node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UASExploration>());
	rclcpp::shutdown();
	return 0;
}