#pragma once
#include "uas_scheduler/Scheduler.h"
#include "uas/UASState.h"
#include "uas/UAS.h"
#include "uas_phases/UASTrailingPhase.h"
#include "uas_phases/UASExplorationPhase.h"
#include "uas_phases/UASCoarseLocalizationPhase.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class CompleteMissionScheduler : public Scheduler
{
    public:
        CompleteMissionScheduler(UAS uas_);

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
        std::unique_ptr<UASCoarseLocalizationPhase> coarsePhase_;

        // methods:
        void timerCallback() override;
        
};