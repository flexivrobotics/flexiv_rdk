#!/bin/bash
set -e
echo "Installing tinyxml2"

# Use a specific version
VER_TAG=8.0.0

# Clone source code
if [ ! -d tinyxml2 ] ; then
  git clone https://github.com/leethomason/tinyxml2.git --branch $VER_TAG
  cd tinyxml2
else
  cd tinyxml2
  git checkout $VER_TAG
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed tinyxml2"
