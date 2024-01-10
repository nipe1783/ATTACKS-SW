#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @brief Listener class to get state variables of the UAS from PX4.
 *
 */
class UASTrailingNode : public rclcpp::Node
{
    public:
        explicit UASTrailingNode();

    private:
        // fields:
        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscriptionState_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscriptionPosition_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriptionPrimarySensor_;
        cv::Mat frame_;
        // methods:

        /**
         * @brief Converts the ROS image message to an OpenCV image.
         *
         * @param msg The ROS image message.
         */   
        void imageConvert(const sensor_msgs::msg::Image::SharedPtr msg);
};