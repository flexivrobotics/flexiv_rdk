#!/bin/bash
# Depends on: foonathan_memory, tinyxml2, Fast-CDR
set -e
repo="Fast-DDS"
echo "Installing $repo"

# Latest v3 version
ver_tag=v3.5.0

# Clone source code
if [ ! -d $repo ] ; then
  git clone --recurse-submodules https://github.com/eProsima/$repo.git --branch $ver_tag
  cd $repo
else
  cd $repo
  git checkout $ver_tag
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS \
         -DTHIRDPARTY_Asio=ON \
         -DCOMPILE_EXAMPLES=OFF \
         -DSQLITE3_SUPPORT=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed $repo"