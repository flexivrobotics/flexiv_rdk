#!/bin/bash
set -e
echo "Installing RBDyn"

# Use a specific version
VER_TAG=v1.9.0

# Clone source code
if [ ! -d RBDyn ] ; then
  git clone https://github.com/jrl-umi3218/RBDyn.git --branch $VER_TAG
  cd RBDyn
else
  cd RBDyn
  git checkout $VER_TAG
fi

# Initialize submodules
git submodule update --init --recursive

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS
         -DPYTHON_BINDING=OFF \
         -DCXX_DISABLE_WERROR=ON

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed RBDyn"
