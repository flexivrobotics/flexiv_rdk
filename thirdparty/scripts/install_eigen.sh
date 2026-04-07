#!/bin/bash
set -e
repo="eigen"
echo "Installing $repo"

# Default version on Ubuntu 22.04
ver_tag=3.4.0

# Clone source code
if [ ! -d $repo ] ; then
  git clone https://gitlab.com/libeigen/$repo.git --branch $ver_tag
  cd $repo
else
  cd $repo
  git checkout $ver_tag
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

echo "Installed $repo"
