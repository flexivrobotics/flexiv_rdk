#!/bin/bash
set -e
echo "Installing foonathan_memory"

# Get install directory as script argument
INSTALL_DIR=$1

# Clone source code
if [ ! -d foonathan_memory_vendor ] ; then
  git clone https://github.com/eProsima/foonathan_memory_vendor.git
  cd foonathan_memory_vendor
else
  cd foonathan_memory_vendor
fi

# Use specific version
git checkout v1.2.1
git submodule update --init --recursive

# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SHARED_LIBS=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR

# Build and install
cmake --build . --target install --config Release -j 4

echo "Installed foonathan_memory"