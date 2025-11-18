#!/bin/bash
set -e
echo "Installing eigen"

# Use a specific version
VER_TAG=3.4.0

# Clone source code
if [ ! -d eigen ] ; then
  git clone https://gitlab.com/libeigen/eigen.git --branch $VER_TAG
  cd eigen
else
  cd eigen
  git checkout $VER_TAG
fi

# Apply patch if building for QNX
git reset --hard
if [ -n "$QNX_TARGET" ]; then
  git apply $SCRIPT_DIR/patches/eigen_qnx802.patch
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed eigen"
