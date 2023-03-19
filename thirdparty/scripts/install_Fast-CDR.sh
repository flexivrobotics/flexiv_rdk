#!/bin/bash
set -e
echo "Installing Fast-CDR"

# Get install directory as script argument
INSTALL_DIR=$1

# Clone source code
if [ ! -d Fast-CDR ] ; then
  git clone https://github.com/eProsima/Fast-CDR.git
  cd Fast-CDR
else
  cd Fast-CDR
fi

# Use specific version
git checkout v1.0.24

# Configure CMake
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_SHARED_LIBS=OFF \
         -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
         -DCOMPILE_EXAMPLES=OFF

# Build and install
cmake --build . --target install --config Release -j 4

echo "Installed Fast-CDR"