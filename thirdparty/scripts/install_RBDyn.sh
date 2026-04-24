#!/usr/bin/env bash
# Depends on: tinyxml2, yaml-cpp, Eigen3, SpaceVecAlg
set -e
repo="RBDyn"
echo "Installing $repo"

# https://github.com/jrl-umi3218/RBDyn/releases/tag/v1.9.3
ver_tag=v1.9.3

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
cmake .. $SHARED_CMAKE_ARGS \
         -DPYTHON_BINDING=OFF \
         -DBoost_USE_STATIC_LIBS=OFF \
         -DCXX_DISABLE_WERROR=ON

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed $repo"
