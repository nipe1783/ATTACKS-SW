cmake_minimum_required(VERSION 3.5)
project(uas_benchmarking)

# Default to C99 and C++17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
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
find_package(uas_computer_vision REQUIRED)
find_package(uas_phases REQUIRED)
find_package(uas REQUIRED)


# Include directories
include_directories(include)

# Add a library
add_library(uas_benchmarking 
  src/UASBenchmarking.cpp
)

# Specify libraries to link a library or executable target against
ament_target_dependencies(uas_benchmarking
  uas
  px4_msgs 
  sensor_msgs 
  cv_bridge 
  Eigen3 
  geometry_msgs 
  OpenCV 
  image_transport
  uas_computer_vision
)

# Install header files and library
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(TARGETS uas_benchmarking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export dependencies and library
ament_export_dependencies(ament_cmake rclcpp eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs cv_bridge OpenCV image_transport uas_computer_vision)
ament_export_include_directories(include)
ament_export_libraries(uas_benchmarking)


# /uasBenchmarking
add_executable(uasBenchmarking src/UASBenchmarking.cpp)
ament_target_dependencies(
    uasBenchmarking
    cv_bridge
    uas
    uas_phases
    uas_computer_vision
    rclcpp 
    px4_msgs 
    sensor_msgs
)
target_link_libraries(uasBenchmarking
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
install(TARGETS uasBenchmarking DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package configuration
ament_package()
