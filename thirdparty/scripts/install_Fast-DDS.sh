#!/bin/bash
# Depends on: foonathan_memory, tinyxml2, Fast-CDR
set -e
echo "Installing Fast-DDS"

# Use a specific version
VER_TAG=v2.6.10

# Clone source code
if [ ! -d Fast-DDS ] ; then
  git clone https://github.com/eProsima/Fast-DDS.git --branch $VER_TAG
  cd Fast-DDS
else
  cd Fast-DDS
  git checkout $VER_TAG
fi

# Initialize submodules
git submodule update --init --recursive

# Apply patch if building for QNX
git reset --hard
if [ -n "$QNX_TARGET" ]; then
  git apply $SCRIPT_DIR/patches/fastdds_qnx802.patch
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS \
         -DTHIRDPARTY_Asio=ON \
         -DCOMPILE_EXAMPLES=OFF \
         -DSQLITE3_SUPPORT=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed Fast-DDS"
