cmake_minimum_required(VERSION 3.0)

project(OpenCVTesting)

# Find the OpenCV package
find_package(OpenCV REQUIRED)

# Add your source files
add_executable(test test.cpp)

# Link the OpenCV libraries to your executable
target_link_libraries(test ${OpenCV_LIBS})

