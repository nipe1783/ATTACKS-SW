#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class UASPhaseNode : public rclcpp::Node
{   
    public:

        // constructors:
        UASPhaseNode(const std::string& node_name) : Node(node_name) { }

        // fields:
        // TODO: uasState_
        cv::Mat psFrame_;
        cv::Mat ssFrame_;
        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscriptionState_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscriptionPosition_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionPS_;

        // methods:
        /**
         * @brief Primary function of all UAS mission phases. Tells the UAS where to go.
         *
         * @param psImg Image from the primary sensor.
         */
        virtual void action(const sensor_msgs::msg::Image::SharedPtr psImg) = 0;

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
        
};