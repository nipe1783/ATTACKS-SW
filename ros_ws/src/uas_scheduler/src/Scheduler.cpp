#include "uas_scheduler/Scheduler.h"
#include "uas/UASState.h"
#include "uas/UAS.h"
#include "uas_helpers/Camera.h"
#include <cv_bridge/cv_bridge.h>
#include <limits>

void Scheduler::imageConvert(const sensor_msgs::msg::Image::SharedPtr sImg)
{
    try {
        psFrame_ = cv_bridge::toCvCopy(sImg, "bgr8")->image;
        psDisplayFrame_ = psFrame_.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", sImg->encoding.c_str());
    }
}

void Scheduler::savePSFrame()
{
    if (!psFrame_.empty()) {
        std::string filename = "saved_image_.jpg";
        cv::imwrite(filename, psFrame_);
        RCLCPP_INFO(this->get_logger(), "Saved image: %s", filename.c_str());
    }
}

double Scheduler::distance(UASState s1, UASState s2)
{
    return sqrt(pow(s1.ix_ - s2.ix_, 2) + pow(s1.iy_ - s2.iy_, 2) + pow(s1.iz_ - s2.iz_, 2));
}

void Scheduler::publishControlMode()
{   
    px4_msgs::msg::OffboardControlMode msg{};
    if(currentPhase_ == "exploration"){
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
    }
    else if(currentPhase_ == "trailing"){
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
    }
    else if(currentPhase_ == "coarse"){
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
    }
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	controlModePublisher_->publish(msg);
}

void Scheduler::publishTrajectorySetpoint(UASState s)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    if(currentPhase_ == "exploration"){
        msg.position = {s.ix_, s.iy_, s.iz_};
        msg.yaw = -3.14;
    }
    else if(currentPhase_ == "trailing"){
        msg.velocity = {s.bxV_, s.byV_, s.bzV_};
        msg.position[0] = std::numeric_limits<float>::quiet_NaN();
        msg.position[1] = std::numeric_limits<float>::quiet_NaN();
        msg.position[2] = std::numeric_limits<float>::quiet_NaN();
        msg.yaw = std::numeric_limits<float>::quiet_NaN();
    }
    else if(currentPhase_ == "coarse"){
        msg.velocity = {s.bxV_, s.byV_, s.bzV_};
        msg.position[0] = std::numeric_limits<float>::quiet_NaN();
        msg.position[1] = std::numeric_limits<float>::quiet_NaN();
        msg.position[2] = std::numeric_limits<float>::quiet_NaN();
        msg.yaw = std::numeric_limits<float>::quiet_NaN();
    }
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectorySetpointPublisher_->publish(msg);
}

void Scheduler::publishVehicleCommand(uint16_t command, float param1, float param2)
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

void Scheduler::arm()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Armed");
}

void Scheduler::disarm()
{
	publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarmed");
}

void Scheduler::callbackPS(const sensor_msgs::msg::Image::SharedPtr psMsg) {
    psMsgReceived_ = true;
    imageConvert(psMsg);
}

void Scheduler::callbackState(const px4_msgs::msg::VehicleLocalPosition::UniquePtr stateMsg) {
    stateMsgReceived_ = true;
    uas_.state_ = UASState(stateMsg->x, stateMsg->y, stateMsg->z, stateMsg->heading, stateMsg->vx,  stateMsg->vy,  stateMsg->vz);
}