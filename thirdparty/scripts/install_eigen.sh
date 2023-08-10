#!/bin/bash
set -e
echo "Installing eigen"

# Get install directory and number of parallel build jobs as script arguments
INSTALL_DIR=$1
NUM_JOBS=$2

# Clone source code
if [ ! -d eigen ] ; then
  git clone https://gitlab.com/libeigen/eigen.git
  cd eigen
else
  cd eigen
fi

# Use specific version
git checkout 3.3.7
git submodule update --init --recursive

# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SHARED_LIBS=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed eigen"
