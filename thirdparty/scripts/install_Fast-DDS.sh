#!/bin/bash
# Depends on: foonathan_memory, tinyxml2, Fast-CDR
set -e
echo "Installing Fast-DDS"

# Clone source code
if [ ! -d Fast-DDS ] ; then
  git clone https://github.com/eProsima/Fast-DDS.git
  cd Fast-DDS
else
  cd Fast-DDS
fi

# Use specific version
git fetch -p
git checkout v2.6.10
git submodule update --init --recursive

# Apply patch if building for QNX
git reset --hard
if [ -n "$QNX_TARGET" ]; then
  git apply $SCRIPTPATH/patches/fastdds_qnx802.patch
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS \
         -DTHIRDPARTY_Asio=ON \
         -DCOMPILE_EXAMPLES=OFF \
         -DSQLITE3_SUPPORT=OFF \
         -DOPENSSL_USE_STATIC_LIBS=ON

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed Fast-DDS"
