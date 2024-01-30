#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include "uas_computer_vision/BasicBlobDetector.h"
#include "uas_computer_vision/Blob.h"
#include "uas/UASState.h"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

class Scheduler : public rclcpp::Node
{   
    public:

        // constructors:
        Scheduler(const std::string& node_name) : Node(node_name) { }

        // fields:
        std::string currentPhase_;
        std::string nextPhase_;
        cv::Mat psFrame_;
        cv::Mat ssFrame_;
        UASState uasState_;
        UASState goalState_;
        BasicBlobDetector blobDetector_;
        CVImg cvImg_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr stateSubscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr psSubscription_;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr controlModePublisher_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectorySetpointPublisher_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicleCommandPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        uint64_t offboardSetpointCounter_;
        bool stateMsgReceived_ = false;
        bool psMsgReceived_ = false;

        // methods:
        /**
         * @brief Function that will allow for the UAS mission to loop. This is where the main logic of the mission will be.
         *
        */
        virtual void timerCallback() = 0;

        /**
         * @brief Converts the ROS vision sensor image message to an OpenCV image.
         *
         * @param msg The ROS image message.
         */
        void imageConvert(const sensor_msgs::msg::Image::SharedPtr sImg);

        /**
         * @brief Saves the last frame from the primary sensor.
         */
        void savePSFrame();

        /**
         * @brief Determines distance between the UAS and a waypoint.
         */
        double distance(UASState, UASState);

        /**
         * @brief Publishes the control mode to the UAS. Allows for the UAS to take in waypoints.
         */
        void publishControlMode();

        /**
         * @brief Publishes the trajectory setpoint to the UAS. Tells the UAS where to go.
         */
        void publishTrajectorySetpoint(UASState uasState);

        /**
         * @brief Publishes a vehicle command to the UAS. Allows for the UAS to arm, disarm, etc.
         */
        void publishVehicleCommand(uint16_t command, float param1, float param2);

        /**
         * @brief Arms the UAS.
         */
        void arm();

        /**
         * @brief Disarms the UAS.
         */
        void disarm();

        /**
         * @brief Converts the ROS vision sensor image message from the ROS topic to an OpenCV image and stores it in psFrame_.
         */
        void callbackPS(const sensor_msgs::msg::Image::SharedPtr msg);

        /**
         * @brief Stores the current UAS state from the ROS topic in uasState_.
         */
        void callbackState(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);
        
};