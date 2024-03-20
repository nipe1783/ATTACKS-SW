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
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class AutonomousTestScheduler : public Scheduler
{
    public:
        AutonomousTestScheduler(UAS uas);

        // fields:
        std::vector<UASState> waypoints_ = {
            UASState(-10, -10, -5, 0, 0, 0, 0), 
            UASState(-10, 10, -5, 0, 0, 0, 0), 
            UASState(10, 10, -5, 0, 0, 0, 0), 
            UASState(10, -10, -5, 0, 0, 0, 0),
            UASState(0, 0, -5, 0, 0, 0, 0)
        };
        unsigned int waypointIndex_;
        std::unique_ptr<UASExplorationPhase> explorationPhase_;
        std::unique_ptr<UASTrailingPhase> trailingPhase_;
        std::unique_ptr<UASCoarseLocalizationPhase> coarsePhase_;

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rgv1StatePublisher_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rgv2StatePublisher_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr missionPhasePublisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

        BasicBlobDetector rgv1BlobDetector_;
        BasicBlobDetector rgv2BlobDetector_;
        RGV rgv1_;
        RGV rgv2_;
        CVImg rgv1CVData_;
        CVImg rgv2CVData_;
        
        // methods:
        void timerCallback() override;
        // void publishRGV1State();
        // void publishRGV2State();
        // void publishMissionPhase();

    private:
        bool takeoff_sent_ = false;
        bool land_sent_ = false;
        void takeoff();
        void land();
        
};