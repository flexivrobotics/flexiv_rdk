#!/usr/bin/env bash
set -e
repo="SpaceVecAlg"
echo "Installing $repo"

# https://github.com/jrl-umi3218/SpaceVecAlg/releases/tag/v1.2.8
ver_tag=v1.2.8

# Clone source code
if [ ! -d $repo ] ; then
  git clone --recurse-submodules https://github.com/jrl-umi3218/$repo.git --branch $ver_tag
  cd $repo
else
  cd $repo
  git checkout $ver_tag
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS -DPYTHON_BINDING=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed $repo"
