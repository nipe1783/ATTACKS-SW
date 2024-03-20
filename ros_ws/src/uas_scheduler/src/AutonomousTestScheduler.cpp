// #include "uas_scheduler/AutonomousTestScheduler.h"
// #include "uas_scheduler/Scheduler.h"
// #include "uas/UAS.h"
// #include "uas_helpers/Camera.h"
// #include "uas_helpers/RGV.h"
// #include "uas_computer_vision/BasicBlobDetector.h"
// #include "uas_computer_vision/Blob.h"
// #include <rclcpp/rclcpp.hpp>
// #include <px4_msgs/msg/sensor_combined.hpp>
// #include <px4_msgs/msg/vehicle_local_position.hpp>
// #include <px4_msgs/msg/vehicle_attitude.hpp>
// #include <px4_msgs/msg/vehicle_command.hpp>
// #include <px4_msgs/msg/vehicle_command_ack.hpp>
// #include <iostream>
// #include <sensor_msgs/msg/image.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <chrono>



// AutonomousTestScheduler::AutonomousTestScheduler(UAS uas) : Scheduler("uas_complete_mission", uas)
// {
//     rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
//     auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
//     psSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//         "/camera/image_raw", qos, std::bind(&AutonomousTestScheduler::callbackPS, this, std::placeholders::_1)
//     );
//     stateSubscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
//         "/fmu/out/vehicle_local_position", qos, std::bind(&AutonomousTestScheduler::callbackState, this, std::placeholders::_1)
//     );
//     attitudeSubscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
//         "/fmu/out/vehicle_attitude", qos, std::bind(&AutonomousTestScheduler::callbackAttitude, this, std::placeholders::_1)
//     );
//     controlModePublisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
//         "/fmu/in/offboard_control_mode", qos
//     );
//     trajectorySetpointPublisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
//         "/fmu/in/trajectory_setpoint", qos
//     );
//     vehicleCommandPublisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
//         "/fmu/in/vehicle_command", qos
//     );
//     rgv1StatePublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
//         "/rgv1/state", qos
//     );
//     rgv2StatePublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
//         "/rgv2/state", qos
//     );
//     missionPhasePublisher_ = this->create_publisher<std_msgs::msg::String>(
//         "/mission/phase", qos
//     );

//     vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);

//     currentPhase_ = "exploration";
//     explorationPhase_ = std::make_unique<UASExplorationPhase>(waypoints_);
//     trailingPhase_ = std::make_unique<UASTrailingPhase>();
//     coarsePhase_ = std::make_unique<UASCoarseLocalizationPhase>();
//     waypointIndex_ = 0;
//     goalState_ = waypoints_[0];
//     offboardSetpointCounter_ = 0;
//     timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&AutonomousTestScheduler::timerCallback, this));
// }
// void AutonomousTestScheduler::timerCallback()
// {
//     // Check if the necessary messages have been received
//     if (!psMsgReceived_ || !stateMsgReceived_)
//     {
//         std::cout << "bye bye" << std::endl;
//         return;
//     }
//     // Send the takeoff command if it hasn't been sent yet
//     if (!takeoff_sent_)
//     {
//         takeoff();
//         std::cout << "TakeOff" << std::endl;
//         takeoff_sent_ = true;
//     }
//     // Send the land command if it hasn't been sent yet
//     if (takeoff_sent_ && !land_sent_)
//     {
//         land();
//         land_sent_ = true;
//     }
//     // // Check for other phases and update goal state accordingly
//     // if (currentPhase_ == "exploration")
//     // {
//     //     std::cout << "Starting Takeoff" << std::endl;
//     //     goalState_ = UASState(0, 0, -5, 0, 0, 0, 0);
//     //     if (uas_.state_.iz_ >= -5.2 && uas_.state_.iz_ <= -4.8)
//     //     {
//     //         currentPhase_ = "trailing";
//     //     }
//     // }
//     // else if (currentPhase_ == "trailing")
//     // {
//     //     std::cout << "Starting Land" << std::endl;
//     //     goalState_ = UASState(0, 0, 0, 0, 0, 0, 0);
//     // }
//     // Publish the goal state for offboard control
//     //publishTrajectorySetpoint(goalState_);
// }
// void AutonomousTestScheduler::takeoff()
// {
//     auto takeoff_cmd = std::make_shared<px4_msgs::msg::VehicleCommand>();
//     takeoff_cmd->command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
//     takeoff_cmd->param1 = 0;
//     takeoff_cmd->param2 = 0;
//     takeoff_cmd->param3 = 0;
//     takeoff_cmd->param4 = 0;
//     takeoff_cmd->param5 = 0;
//     takeoff_cmd->param6 = 0;
//     takeoff_cmd->param7 = 0;
//     vehicle_command_pub_->publish(*takeoff_cmd);

// }
// void AutonomousTestScheduler::land()
// {
//     auto land_cmd = std::make_shared<px4_msgs::msg::VehicleCommand>();
//     land_cmd->command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
//     land_cmd->param1 = 0;
//     land_cmd->param2 = 0;
//     land_cmd->param3 = 0;
//     land_cmd->param4 = 0;
//     land_cmd->param5 = 0;
//     land_cmd->param6 = 0;
//     land_cmd->param7 = 0;
//     vehicle_command_pub_->publish(*land_cmd);

// }


// int main(int argc, char *argv[])
// {
// 	std::cout << "Starting UAS autonomous test node..." << std::endl;
//     // RGV rgv1 = RGV(1, 0, 30, 180, 255, 180, 255); // RED
//     // RGV rgv2 = RGV(2, 0, 10, 0, 10, 180, 255); // WHITE
// 	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
// 	rclcpp::init(argc, argv);
//     UAS missionUAS = UAS();
//     // Camera camera1 = Camera(640, 360, 2, 1.125);
//     // missionUAS.addCamera(camera1);
// 	rclcpp::spin(std::make_shared<AutonomousTestScheduler>(missionUAS));
// 	rclcpp::shutdown();
// 	return 0;
// }