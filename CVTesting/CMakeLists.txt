cmake_minimum_required(VERSION 3.10)
project(ComputerVision)
find_package(OpenCV 4 REQUIRED)

add_executable(
    ${PROJECT_NAME} 
    main.cpp
    blobDetection/BlobDetection.cpp
)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} "${TORCH_LIBRARIES}")