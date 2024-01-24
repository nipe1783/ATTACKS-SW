#pragma once
#include "uas_lib/uas_mission.h"
#include "uas_lib/UASState.h"
#include "uas_lib/UASExplorationPhase.h"
#include "uas_lib/UASTrailingPhase.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class UASCompleteMission : public UASMission
{
    public:
        UASCompleteMission();

        // fields:
        std::vector<UASState> waypoints_ = {
            UASState(-10, -10, -5, 0, 0, 0, 0), 
            UASState(-10, 10, -5, 0, 0, 0, 0), 
            UASState(10, 10, -5, 0, 0, 0, 0), 
            UASState(10, -10, -5, 0, 0, 0, 0),
        };
        unsigned int waypointIndex_;
        std::unique_ptr<UASExplorationPhase> explorationPhase_;
        std::unique_ptr<UASTrailingPhase> trailingPhase_;

        // methods:
        void timerCallback() override;
        
};