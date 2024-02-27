#pragma once
#include "uas_scheduler/Scheduler.h"
#include "uas/UASState.h"
#include "uas/UAS.h"
#include "uas_helpers/RGV.h"
#include "uas_phases/UASTrailingPhase.h"
#include "uas_phases/UASExplorationPhase.h"
#include "uas_phases/UASCoarseLocalizationPhase.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <chrono>

class CompleteMissionScheduler : public Scheduler
{
    public:
        CompleteMissionScheduler(UAS uas, RGV rgv1, RGV rgv2);

        // fields:
        std::vector<UASState> waypoints_ = {
            UASState(-10, -10, -10, 0, 0, 0, 0), 
            UASState(-10, 10, -10, 0, 0, 0, 0), 
            UASState(10, 10, -10, 0, 0, 0, 0), 
            UASState(10, -10, -10, 0, 0, 0, 0),
            UASState(0, 0, -10, 0, 0, 0, 0)
        };
        unsigned int waypointIndex_;
        std::unique_ptr<UASExplorationPhase> explorationPhase_;
        std::unique_ptr<UASTrailingPhase> trailingPhase_;
        std::unique_ptr<UASCoarseLocalizationPhase> coarsePhase_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rgv1StatePublisher_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rgv2StatePublisher_;

        BasicBlobDetector rgv1BlobDetector_;
        BasicBlobDetector rgv2BlobDetector_;
        RGV rgv1_;
        RGV rgv2_;
        CVImg rgv1CVData_;
        CVImg rgv2CVData_;

        float maxHeight_ = -0.3048*60.0;
        float minHeight_ = -0.3048*30.0;

        std::chrono::steady_clock::time_point uasStoppedTime_;
        bool uasStopped_;
        float stopVelocityThresh_;
        
        // methods:
        void checkUASStopped();
        void timerCallback() override;
        void publishRGV1State();
        void publishRGV2State();
        
};