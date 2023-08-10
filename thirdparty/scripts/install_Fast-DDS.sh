#!/bin/bash
# Depends on: foonathan_memory, tinyxml2, Fast-CDR
set -e
echo "Installing Fast-DDS"

# Get install directory and number of parallel build jobs as script arguments
INSTALL_DIR=$1
NUM_JOBS=$2

# Clone source code
if [ ! -d Fast-DDS ] ; then
  git clone https://github.com/eProsima/Fast-DDS.git
  cd Fast-DDS
else
  cd Fast-DDS
fi

# Use specific version
git checkout v2.6.2
git submodule update --init --recursive

# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SHARED_LIBS=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
         -DTHIRDPARTY_Asio=ON \
         -DCOMPILE_EXAMPLES=OFF \
         -DSQLITE3_SUPPORT=OFF \
         -DOPENSSL_USE_STATIC_LIBS=ON

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed Fast-DDS"
