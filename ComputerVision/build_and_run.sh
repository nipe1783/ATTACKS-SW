#!/bin/bash
BUILD_DIR="./build"
cd "$BUILD_DIR"
if [ $? -ne 0 ]; then
    echo "Failed to navigate to the build directory."
    exit 1
fi
make clean
cmake ..
make
./BlobDetectionPOC