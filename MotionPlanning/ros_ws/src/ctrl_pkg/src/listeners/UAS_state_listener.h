#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

/**
 * @brief Listener class to get state variables of the UAS from PX4.
 *
 */
class UasStateListener : public rclcpp::Node
{
    public:
        explicit UasStateListener();

    private:
        rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscriptionState_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscriptionPosition_;
};