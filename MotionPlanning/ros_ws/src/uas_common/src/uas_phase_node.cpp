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