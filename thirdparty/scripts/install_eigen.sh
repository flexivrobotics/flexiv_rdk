#!/bin/bash
set -e
echo "Installing eigen"

URL=https://gitlab.com/libeigen/eigen.git
# Same as the apt installed version on Ubuntu 22.04
VER_TAG=3.4.0
# Commit VER_TAG pointed to when it was pinned. The sources are checked out by this commit rather
# than by the tag, because a tag can be moved upstream and would then silently change the build.
VER_SHA=3147391d946bb4b6c68edd901f2add6ac1f31f8c

# Clone source code
if [ ! -d eigen ] ; then
  git clone $URL --branch $VER_TAG
  cd eigen
else
  cd eigen
  git fetch -p --tags
fi

# Check out the pinned commit and remove everything a previous run left behind (local changes,
# applied patches, build outputs), so the build always starts from exactly the pinned sources
git checkout --force "$VER_SHA"
git clean -xfdf
if [ "$(git rev-parse HEAD)" != "$VER_SHA" ]; then
  echo "Error: eigen is at $(git rev-parse HEAD) instead of the pinned commit $VER_SHA ($VER_TAG)"
  exit 1
fi

# Apply patch if building for QNX
if [ -n "$QNX_TARGET" ]; then
  git apply $SCRIPT_DIR/patches/eigen_qnx802.patch
fi

# Configure CMake
rm -rf build && mkdir build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed eigen"
