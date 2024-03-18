cmake_minimum_required(VERSION 3.5)
project(uas_scheduler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(uas REQUIRED)
find_package(uas_phases REQUIRED)
find_package(uas_computer_vision REQUIRED)
find_package(uas_helpers REQUIRED)

#################
# Setup targets #
#################

include_directories(include)

# /completeMissionScheduler
add_executable(completeMissionScheduler src/CompleteMissionScheduler.cpp src/Scheduler.cpp)
ament_target_dependencies(
    completeMissionScheduler
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    uas_helpers
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(completeMissionScheduler
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(TARGETS completeMissionScheduler DESTINATION lib/${PROJECT_NAME})


# /autonomousTestScheduler
add_executable(autonomousTestScheduler src/AutonomousTestScheduler.cpp src/Scheduler.cpp)
ament_target_dependencies(
    autonomousTestScheduler
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    uas_helpers
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(autonomousTestScheduler
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(TARGETS autonomousTestScheduler DESTINATION lib/${PROJECT_NAME})


# /cameraCalibrateMissionScheduler
add_executable(cameraCalibrateMissionScheduler src/CameraCalibrateMissionScheduler.cpp src/Scheduler.cpp)
ament_target_dependencies(
    cameraCalibrateMissionScheduler
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    uas_helpers
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(cameraCalibrateMissionScheduler
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(TARGETS cameraCalibrateMissionScheduler DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_include_directories(include)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_package()
