cmake_minimum_required(VERSION 3.10)
project(ComputerVision)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_PREFIX_PATH /home/nicolas/repos/libtorch)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(OpenCV 4 REQUIRED)


add_executable(
    ${PROJECT_NAME} 
    main.cpp
    blobDetector/src/BlobDetector.cpp
    blobDetector/src/BasicBlobDetector.cpp
    blobDetector/src/VaryingLightBlobDetector.cpp
    visualizer/Visualizer.cpp
    blob/Blob.cpp
    benchmarking/Benchmarking.cpp
    scripts/Scripts.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} "${TORCH_LIBRARIES}")
