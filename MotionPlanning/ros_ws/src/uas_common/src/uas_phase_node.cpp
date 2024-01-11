#include "uas_common/uas_phase_node.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <iostream>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

void UASPhaseNode::imageConvert(const sensor_msgs::msg::Image::SharedPtr sImg)
{
    try {
        psFrame_ = cv_bridge::toCvCopy(sImg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", sImg->encoding.c_str());
    }
}

void UASPhaseNode::savePSFrame()
{
    if (!psFrame_.empty()) {
        std::string filename = "saved_image_.jpg";
        cv::imwrite(filename, psFrame_);
        RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
    }
}

double UASPhaseNode::distance(Waypoint wp1, Waypoint wp2)
{
    return sqrt(pow(wp1.x - wp2.x, 2) + pow(wp1.y - wp2.y, 2) + pow(wp1.z - wp2.z, 2));
}

void UASPhaseNode::publishControlMode()
{   
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	controlModePublisher_->publish(msg);
}

void UASPhaseNode::publishTrajectorySetpoint(Waypoint wp)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {wp.x, wp.y, wp.z};
    msg.yaw = -3.14;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectorySetpointPublisher_->publish(msg);
}

void UASPhaseNode::publishVehicleCommand(uint16_t command, float param1, float param2)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicleCommandPublisher_->publish(msg);
}

void UASPhaseNode::arm()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Armed");
}

void UASPhaseNode::callbackPS(const sensor_msgs::msg::Image::SharedPtr psMsg) {
    psMsgReceived_ = true;
    imageConvert(psMsg);
}

void UASPhaseNode::callbackState(const px4_msgs::msg::VehicleLocalPosition::UniquePtr stateMsg) {
    stateMsgReceived_ = true;
    uasPosition_ = Waypoint(stateMsg->x, stateMsg->y, stateMsg->z);
}