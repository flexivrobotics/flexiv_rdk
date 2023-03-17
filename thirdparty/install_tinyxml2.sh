#!/bin/bash
set -e
echo "Installing tinyxml2"

# Get install directory as script argument
INSTALL_DIR=$1

# Clone source code
if [ ! -d tinyxml2 ] ; then
  git clone https://github.com/leethomason/tinyxml2.git
  cd tinyxml2
else
  cd tinyxml2
fi

# Use specific version
git checkout 8.0.0

# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SHARED_LIBS=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \

# Build and install
cmake --build . --target install --config Release -j 4

echo "Installed tinyxml2"