#include "UAS_state_listener.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <iostream>

UasStateListener::UasStateListener() : Node("uas_state_listener")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    subscriptionState_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
    [this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
        std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
        std::cout << "RECEIVED SENSOR DATA"   << std::endl;
        std::cout << "============================="   << std::endl;
        std::cout << "time stamp: "          << msg->timestamp    << std::endl;
        std::cout << "gyro_rad[0]: " << msg->gyro_rad[0]  << std::endl;
        std::cout << "gyro_rad[1]: " << msg->gyro_rad[1]  << std::endl;
        std::cout << "gyro_rad[2]: " << msg->gyro_rad[2]  << std::endl;
        std::cout << "gyro_integral_dt: " << msg->gyro_integral_dt << std::endl;
        std::cout << "accelerometer_timestamp_relative: " << msg->accelerometer_timestamp_relative << std::endl;
        std::cout << "accelerometer_m_s2[0]: " << msg->accelerometer_m_s2[0] << std::endl;
        std::cout << "accelerometer_m_s2[1]: " << msg->accelerometer_m_s2[1] << std::endl;
        std::cout << "accelerometer_m_s2[2]: " << msg->accelerometer_m_s2[2] << std::endl;
        std::cout << "accelerometer_integral_dt: " << msg->accelerometer_integral_dt << std::endl;
    });

    subscriptionPosition_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos,
    [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
        std::cout << "local x: " << msg->x << std::endl;
        std::cout << "local y: " << msg->y << std::endl;
        std::cout << "local z: " << msg->z << std::endl;
    });
}

int main(int argc, char *argv[])
{
	std::cout << "Starting uas_state listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UasStateListener>());

	rclcpp::shutdown();
	return 0;
}
