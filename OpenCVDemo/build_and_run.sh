#!/bin/bash

# Define the build directory relative to this script
BUILD_DIR="./build"

# Navigate to the build directory
cd "$BUILD_DIR"

# If navigation failed, exit
if [ $? -ne 0 ]; then
    echo "Failed to navigate to the build directory."
    exit 1
fi

# Run 'make clean'
make clean

# Run 'cmake ..'
cmake ..

# Run 'make'
make

# Execute the OpenCVExample program
./OpenCVExample
