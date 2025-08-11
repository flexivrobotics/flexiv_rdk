#!/bin/bash
set -e
echo "Installing eigen"

# Clone source code
if [ ! -d eigen ] ; then
  git clone https://gitlab.com/libeigen/eigen.git
  cd eigen
else
  cd eigen
fi

# Use specific version
git fetch -p
git checkout 3.4.0

# Apply patch if building for QNX
git reset --hard
if [ -n "$QNX_TARGET" ]; then
  git apply $SCRIPTPATH/patches/eigen_qnx802.patch
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed eigen"
