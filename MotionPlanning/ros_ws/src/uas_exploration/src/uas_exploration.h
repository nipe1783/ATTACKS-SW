#pragma once
#include "uas_common/uas_phase_node.h"
#include "uas_common/Waypoint.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class UASExploration : public UASPhaseNode
{
    public:
        UASExploration();

        // fields:
        std::vector<Waypoint> waypoints_ = {
            Waypoint(-10, -10, -5.0), 
            Waypoint(-10, 10, -5), 
            Waypoint(10, 10, -5) , 
            Waypoint(10, -10, -5)
        };
        unsigned int waypointIndex_;

        // methods:
        void action() override;
};