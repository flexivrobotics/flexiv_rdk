#!/usr/bin/env bash
set -e
repo="tinyxml2"
echo "Installing $repo"

# Default version on Ubuntu 22.04
ver_tag=9.0.0

# Clone source code
if [ ! -d $repo ] ; then
  git clone https://github.com/leethomason/$repo.git --branch $ver_tag
  cd $repo
else
  cd $repo
  git checkout $ver_tag
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed $repo"
