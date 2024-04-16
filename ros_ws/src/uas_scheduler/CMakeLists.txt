cmake_minimum_required(VERSION 3.5)
project(uas_scheduler)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    set(FILESYSTEM_LIBRARY "stdc++fs")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    set(FILESYSTEM_LIBRARY "c++fs")
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
find_package(yaml-cpp REQUIRED)

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
  yaml-cpp
)
install(TARGETS completeMissionScheduler DESTINATION lib/${PROJECT_NAME})

# /hoverMissionScheduler
add_executable(hoverMissionScheduler src/HoverMissionScheduler.cpp src/Scheduler.cpp)
ament_target_dependencies(
    hoverMissionScheduler
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    uas_helpers
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(hoverMissionScheduler
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
install(TARGETS hoverMissionScheduler DESTINATION lib/${PROJECT_NAME})

# /waypointMissionScheduler
add_executable(waypointMissionScheduler src/WaypointMissionScheduler.cpp src/Scheduler.cpp)
ament_target_dependencies(
    waypointMissionScheduler
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    uas_helpers
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(waypointMissionScheduler
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
install(TARGETS waypointMissionScheduler DESTINATION lib/${PROJECT_NAME})

# /trailingMissionScheduler
add_executable(trailingMissionScheduler src/TrailingMissionScheduler.cpp src/Scheduler.cpp)
ament_target_dependencies(
    trailingMissionScheduler
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    uas_helpers
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(trailingMissionScheduler
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
install(TARGETS trailingMissionScheduler DESTINATION lib/${PROJECT_NAME})


# /trailingTestMissionScheduler
add_executable(trailingTestMissionScheduler src/TrailingTestMissionScheduler.cpp src/Scheduler.cpp)
ament_target_dependencies(
    trailingTestMissionScheduler
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    uas_helpers
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(trailingTestMissionScheduler
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
install(TARGETS trailingTestMissionScheduler DESTINATION lib/${PROJECT_NAME})


# /completeTestMissionScheduler
add_executable(completeTestMissionScheduler src/CompleteTestMissionScheduler.cpp src/Scheduler.cpp)
ament_target_dependencies(
    completeTestMissionScheduler
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    uas_helpers
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(completeTestMissionScheduler
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)
install(TARGETS completeTestMissionScheduler DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs)

ament_export_include_directories(include)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_package()
