#!/bin/bash
set -e
repo="yaml-cpp"
echo "Installing $repo"

# Default version on Ubuntu 22.04
ver_tag=yaml-cpp-0.7.0

# Clone source code
if [ ! -d $repo ] ; then
  git clone https://github.com/jbeder/$repo.git --branch $ver_tag
  cd $repo
else
  cd $repo
  git checkout $ver_tag
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS -DYAML_CPP_BUILD_TESTS=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed $repo"
