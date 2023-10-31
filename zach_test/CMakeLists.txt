cmake_minimum_required(VERSION 3.12)
project(OpenCVTest)

# Set C++ standard (e.g., C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Add your test source files
add_executable(test_main test_main.cpp)

# Link OpenCV to your test project
target_link_libraries(test_main ${OpenCV_LIBS})