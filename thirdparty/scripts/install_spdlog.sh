#!/bin/bash
set -e
echo "Installing spdlog"

# Clone source code
if [ ! -d spdlog ] ; then
  git clone https://github.com/gabime/spdlog.git
  cd spdlog
else
  cd spdlog
fi

# Use specific version
git fetch -p
git checkout v1.14.1

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS \
         -DSPDLOG_BUILD_EXAMPLE=OFF \
         -DSPDLOG_BUILD_TESTS=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed spdlog"
