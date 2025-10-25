#!/bin/bash
set -e
echo "Installing Fast-CDR"

# Use a specific version
VER_TAG=v1.0.28

# Clone source code
if [ ! -d Fast-CDR ] ; then
  git clone https://github.com/eProsima/Fast-CDR.git --branch $VER_TAG
  cd Fast-CDR
else
  cd Fast-CDR
  git checkout $VER_TAG
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS -DCOMPILE_EXAMPLES=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed Fast-CDR"
