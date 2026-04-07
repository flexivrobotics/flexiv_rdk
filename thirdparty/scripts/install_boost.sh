#!/bin/bash
set -e
repo="boost"
echo "Installing $repo"

# Default version on Ubuntu 24.04
# Also the minimum version compatible with MSVC v143
ver_tag=1.83.0
ver_str=1_83_0

# Download source code, 1.83 
if [ ! -d ${repo}_${ver_str} ] ; then
  # download is faster than clone
  URL="https://archives.boost.io/release/$ver_tag/source/${repo}_${ver_str}.tar.bz2"
  echo "-- Downloading: $URL"
  if [[ "$OSTYPE" == "msys"* ]]; then # Windows
    curl -L -o ${repo}_${ver_str}.tar.bz2 $URL
  else
    wget $URL --no-clobber --quiet --show-progress --progress=bar:force 2>&1
  fi
  # Unzip
  echo "-- Extracting: ${repo}_${ver_str}.tar.bz2"
  tar --bzip2 -xf "${repo}_${ver_str}.tar.bz2"
  cd ${repo}_${ver_str}
else
  cd ${repo}_${ver_str}
fi

# Bootstrap differently on Unix and Windows
if [[ "$OSTYPE" == "msys"* ]]; then
  # Windows
  ./bootstrap.bat
else
  # Unix
  ./bootstrap.sh
fi

# Build differently on QNX and non-QNX
if [ -n "$QNX_TARGET" ]; then
  if [ $QNX_ARCH == "x86_64" ]; then
    # Build for x86_64 target
    echo "Building $repo for QNX x86_64"
    ./b2 -j$NUM_JOBS --prefix=$INSTALL_DIR --with-system --with-filesystem variant=release link=shared \
    toolset=qcc target-os=qnx architecture=x86 address-model=64 \
    cxxflags="-Vgcc_ntox86_64" linkflags="-Vgcc_ntox86_64" install
  elif [ $QNX_ARCH == "aarch64" ]; then
    # Build for aarch64 target
    echo "Building $repo for QNX aarch64"
    ./b2 -j$NUM_JOBS --prefix=$INSTALL_DIR --with-system --with-filesystem variant=release link=shared \
    toolset=qcc target-os=qnx architecture=arm address-model=64 \
    cxxflags="-Vgcc_ntoaarch64le" linkflags="-Vgcc_ntoaarch64le" install
  else
    echo "Error: unknown QNX platform architecture"
    exit 1
  fi
else
  # Build for non-QNX
  ./b2 -j$NUM_JOBS --prefix=$INSTALL_DIR --with-system --with-filesystem variant=release link=shared install
fi

echo "Installed $repo"
