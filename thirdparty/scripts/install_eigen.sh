#!/usr/bin/env bash
set -e
repo="eigen"
echo "Installing $repo"
script_dir="$(dirname $(readlink -f $0))"

# Default version on Ubuntu 22.04
ver_tag=3.4.0
# Commit [ver_tag] pointed to when it was pinned. The sources are checked out by this commit
# rather than by the tag, because a tag can be moved upstream and would then silently change
# the build.
ver_sha=3147391d946bb4b6c68edd901f2add6ac1f31f8c

# Clone source code
if [ ! -d $repo ] ; then
  git clone https://gitlab.com/libeigen/$repo.git --branch $ver_tag
  cd $repo
else
  cd $repo
  git fetch --all --tags
fi

# Check out the pinned commit and remove everything a previous run left behind (local changes,
# build outputs), so the build always starts from exactly the pinned sources
git checkout --force "$ver_sha"
git clean -xfdf
if [ "$(git rev-parse HEAD)" != "$ver_sha" ]; then
  echo "Error: $repo is at $(git rev-parse HEAD) instead of the pinned commit $ver_sha ($ver_tag)"
  exit 1
fi

# Configure CMake
rm -rf build && mkdir build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed $repo"
