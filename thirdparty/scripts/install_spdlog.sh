#!/bin/bash
set -e
repo="spdlog"
echo "Installing $repo"

# Default version on Ubuntu 22.04
ver_tag=v1.9.2

# Clone source code
if [ ! -d $repo ] ; then
  git clone https://github.com/gabime/$repo.git --branch $ver_tag
  cd $repo
else
  cd $repo
  git checkout $ver_tag
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS \
         -DSPDLOG_BUILD_EXAMPLE=OFF \
         -DSPDLOG_BUILD_TESTS=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed $repo"
