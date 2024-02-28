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
#include <yaml-cpp/yaml.h>


class CompleteMissionScheduler : public Scheduler
{
    public:
        CompleteMissionScheduler(std::string configPath);

        // fields:
        std::vector<UASState> waypoints_;
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
        Camera camera1_;
        Camera camera2_;
        CVImg rgv1CVData_;
        CVImg rgv2CVData_;
        float maxHeight_;
        float minHeight_;
        float stopVelocityThresh_;
        int stopTimeThresh_;
        int coarseLocalizationTime_;
        int fineLocalizationTime_;
        int waypointWaitDuration_;
        int maxWaypointWaitDuration_;
        float minWaypointDistanceThresh_;

        std::chrono::time_point<std::chrono::system_clock> waypointDelayStart_;
        
        // methods:
        bool isUASStopped(RGV rgv);
        bool isRGVCoarseLocalized(RGV rgv);
        bool isRGVNearCenter(CVImg cvImg);
        void timerCallback() override;
        void publishRGV1State();
        void publishRGV2State();
        
};