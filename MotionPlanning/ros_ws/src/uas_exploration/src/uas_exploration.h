#pragma once
#include "uas_common/uas_phase_node.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class UASExploration : public UASPhaseNode
{
    public:
        UASExploration();

        // fields:


        // methods:
        void action(const sensor_msgs::msg::Image::SharedPtr psImg) override;
        
};