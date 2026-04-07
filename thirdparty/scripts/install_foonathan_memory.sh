#!/bin/bash
set -e
repo="foonathan_memory_vendor"
echo "Installing $repo"

# Version listed in https://github.com/eProsima/Fast-DDS/blob/v3.5.0/fastdds.repos
ver_tag=v1.4.1

# Clone source code
if [ ! -d $repo ] ; then
  git clone https://github.com/eProsima/$repo.git --branch $ver_tag
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
