#!/bin/bash
set -e
echo "Installing tinyxml2"

# Get install directory and number of parallel build jobs as script arguments
INSTALL_DIR=$1
NUM_JOBS=$2

# Clone source code
if [ ! -d tinyxml2 ] ; then
  git clone https://github.com/leethomason/tinyxml2.git
  cd tinyxml2
else
  cd tinyxml2
fi

# Use specific version
git fetch -p
git checkout 8.0.0
git submodule update --init --recursive

# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SHARED_LIBS=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
         -DCMAKE_PREFIX_PATH=$INSTALL_DIR

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed tinyxml2"
