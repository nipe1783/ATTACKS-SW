#include "uas_exploration.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <iostream>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>

UASExploration::UASExploration() : UASPhaseNode("uas_exploration_node") {

    subscriptionPS_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&UASExploration::action, this, std::placeholders::_1)
    );

}

void UASExploration::action(const sensor_msgs::msg::Image::SharedPtr psImg) {
    imageConvert(psImg);
    cv::imshow("Primary Sensor", psFrame_);
    int key = cv::waitKey(0);
    if (key == 's') {
        savePSFrame();
    }

    
}

int main(int argc, char *argv[])
{
	std::cout << "Starting UAS exploration node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<UASExploration>());
	rclcpp::shutdown();
	return 0;
}