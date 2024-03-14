#include "uas_scheduler/Scheduler.h"
#include "uas/UASState.h"
#include "uas/UAS.h"
#include "uas_helpers/Camera.h"
#include <cv_bridge/cv_bridge.h>
#include <limits>

void Scheduler::imageConvertPS(const sensor_msgs::msg::Image::SharedPtr sImg)
{
    try {
        psFrame_ = cv_bridge::toCvCopy(sImg, "bgr8")->image;
        psDisplayFrame_ = psFrame_.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", sImg->encoding.c_str());
    }
}

void Scheduler::imageConvertSS(const sensor_msgs::msg::Image::SharedPtr sImg)
{
    try {
        ssFrame_ = cv_bridge::toCvCopy(sImg, "bgr8")->image;
        ssDisplayFrame_ = ssFrame_.clone();
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
    else if(currentPhase_ == "trailing" || currentPhase_ == "jointExploration" || currentPhase_ == "jointTrailing"){
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
    msg.timestamp = timestamp_.load();
	controlModePublisher_->publish(msg);
}

void Scheduler::publishTrajectorySetpoint(UASState s)
{
    px4_msgs::msg::TrajectorySetpoint msg{};
    if(currentPhase_ == "exploration"){
        msg.x = s.ix_;
        msg.y = s.iy_;
        msg.z = s.iz_;
        msg.yaw = -3.14; 
    }
    else if(currentPhase_ == "trailing" || currentPhase_ == "jointExploration" || currentPhase_ == "jointTrailing"){
        msg.set__vx(s.bxV_);
        msg.set__vy(s.byV_);
        msg.set__vz(s.bzV_);
        msg.set__x(std::numeric_limits<float>::quiet_NaN());
        msg.set__y(std::numeric_limits<float>::quiet_NaN());
        msg.set__z(std::numeric_limits<float>::quiet_NaN());
        msg.set__yaw(std::numeric_limits<float>::quiet_NaN());
    }
    else if(currentPhase_ == "coarse"){
        msg.set__vx(s.bxV_);
        msg.set__vy(s.byV_);
        msg.set__vz(s.bzV_);
        msg.set__x(std::numeric_limits<float>::quiet_NaN());
        msg.set__y(std::numeric_limits<float>::quiet_NaN());
        msg.set__z(std::numeric_limits<float>::quiet_NaN());
        msg.set__yaw(std::numeric_limits<float>::quiet_NaN());
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
    imageConvertPS(psMsg);
}

void Scheduler::callbackSS(const sensor_msgs::msg::Image::SharedPtr ssMsg) {
    ssMsgReceived_ = true;
    imageConvertSS(ssMsg);
}

void Scheduler::callbackState(const px4_msgs::msg::VehicleLocalPosition::UniquePtr stateMsg) {
    stateMsgReceived_ = true;
    uas_.state_.updateState(stateMsg->x, stateMsg->y, stateMsg->z, stateMsg->heading, stateMsg->vx,  stateMsg->vy,  stateMsg->vz);
}

void Scheduler::callbackAttitude(const px4_msgs::msg::VehicleAttitude::UniquePtr attMsg) {
    uas_.state_.updateAttitude(attMsg->q[0],attMsg->q[1],attMsg->q[2],attMsg->q[3]);
}