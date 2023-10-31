#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <DirectoryName>"
    exit 1
fi

DIRECTORY="$1"
BUILD_DIR="./build"

# Navigate to the input directory
cd "$DIRECTORY"
if [ $? -ne 0 ]; then
    echo "Failed to navigate to the directory $DIRECTORY."
    exit 1
fi

# Now navigate to the build directory inside the input directory
cd "$BUILD_DIR"
if [ $? -ne 0 ]; then
    echo "Failed to navigate to the build directory inside $DIRECTORY."
    exit 1
fi

make clean
cmake ..
make

# Run the executable
./"$DIRECTORY"