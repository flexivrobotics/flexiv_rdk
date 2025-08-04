#!/bin/bash
set -e
echo "Installing tinyxml2"

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

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed tinyxml2"
