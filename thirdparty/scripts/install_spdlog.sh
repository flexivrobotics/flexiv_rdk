#!/bin/bash
set -e

# Use a specific version
VER_TAG=v1.9.2

# Clone source code
if [ ! -d spdlog ] ; then
  git clone https://github.com/gabime/spdlog.git --branch $VER_TAG
  cd spdlog
else
  cd spdlog
  git checkout $VER_TAG
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS \
         -DSPDLOG_BUILD_EXAMPLE=OFF \
         -DSPDLOG_BUILD_TESTS=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed spdlog"
