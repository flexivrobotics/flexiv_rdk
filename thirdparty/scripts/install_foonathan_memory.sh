#!/bin/bash
set -e
echo "Installing foonathan_memory"

# Use a specific version
VER_TAG=v1.3.1

# Clone source code
if [ ! -d foonathan_memory_vendor ] ; then
  git clone https://github.com/eProsima/foonathan_memory_vendor.git --branch $VER_TAG
  cd foonathan_memory_vendor
else
  cd foonathan_memory_vendor
  git checkout $VER_TAG
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed foonathan_memory"
