#!/bin/bash
set -e
echo "Installing yaml-cpp"

# Same as the apt installed version on Ubuntu 22.04
VER_TAG=yaml-cpp-0.7.0

# Clone source code
if [ ! -d yaml-cpp ] ; then
  git clone https://github.com/jbeder/yaml-cpp.git --branch $VER_TAG
  cd yaml-cpp
else
  cd yaml-cpp
  git checkout $VER_TAG
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS -DYAML_CPP_BUILD_TESTS=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed yaml-cpp"
