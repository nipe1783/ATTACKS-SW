#pragma once
#include "uas_scheduler/Scheduler.h"
#include "uas/UASState.h"
#include "uas/UAS.h"
#include "uas_helpers/RGV.h"
#include "uas_phases/UASTrailingPhase.h"
#include "uas_phases/UASExplorationPhase.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class CompleteMissionScheduler : public Scheduler
{
    public:
        CompleteMissionScheduler(UAS uas, RGV rgv1, RGV rgv2);

        // fields:
        std::vector<UASState> waypoints_ = {
            // UASState(-10, -10, -5, 0, 0, 0, 0), 
            // UASState(-10, 10, -5, 0, 0, 0, 0), 
            // UASState(10, 10, -5, 0, 0, 0, 0), 
            // UASState(10, -10, -5, 0, 0, 0, 0),
            UASState(0, 0, -5, 0, 0, 0, 0)
        };
        unsigned int waypointIndex_;
        std::unique_ptr<UASExplorationPhase> explorationPhase_;
        std::unique_ptr<UASTrailingPhase> trailingPhase_;
        BasicBlobDetector rgv1BlobDetector_;
        BasicBlobDetector rgv2BlobDetector_;
        RGV rgv1_;
        RGV rgv2_;
        CVImg rgv1CVData_;
        CVImg rgv2CVData_;
        
        // methods:
        void timerCallback() override;
        
};