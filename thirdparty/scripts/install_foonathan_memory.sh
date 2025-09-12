#!/bin/bash
set -e
echo "Installing foonathan_memory"

# Clone source code
if [ ! -d foonathan_memory_vendor ] ; then
  git clone https://github.com/eProsima/foonathan_memory_vendor.git
  cd foonathan_memory_vendor
else
  cd foonathan_memory_vendor
fi

# Use specific version
git fetch -p
git checkout v1.3.1

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed foonathan_memory"
