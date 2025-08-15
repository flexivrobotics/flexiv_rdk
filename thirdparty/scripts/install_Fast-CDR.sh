#!/bin/bash
set -e
echo "Installing Fast-CDR"

# Clone source code
if [ ! -d Fast-CDR ] ; then
  git clone https://github.com/eProsima/Fast-CDR.git
  cd Fast-CDR
else
  cd Fast-CDR
fi

# Use specific version
git fetch -p
git checkout v1.0.28

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS -DCOMPILE_EXAMPLES=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed Fast-CDR"
