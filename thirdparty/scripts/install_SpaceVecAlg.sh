#!/bin/bash
set -e
echo "Installing SpaceVecAlg"

# Use a specific version
VER_TAG=v1.2.4

# Clone source code
if [ ! -d SpaceVecAlg ] ; then
  git clone https://github.com/jrl-umi3218/SpaceVecAlg.git --branch $VER_TAG
  cd SpaceVecAlg
else
  cd SpaceVecAlg
  git checkout $VER_TAG
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS -DPYTHON_BINDING=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed SpaceVecAlg"
