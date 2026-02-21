#!/bin/bash
set -e
echo "Installing boost"

# Same as the apt installed version on Ubuntu 24.04 
# Also the minimum version compatible with MSVC v143
VER_TAG=1.83.0
VER_STR=1_83_0

# Download source code, 1.83 
if [ ! -d boost_$VER_STR ] ; then
  # download is faster than clone
  URL="https://archives.boost.io/release/$VER_TAG/source/boost_$VER_STR.tar.bz2"
  echo "-- Downloading: $URL"
  if [ $OS_NAME == "Windows" ]; then
    curl -L -o boost_$VER_STR.tar.bz2 $URL
  else
    wget $URL --no-clobber --quiet --show-progress --progress=bar:force 2>&1
  fi
  # Unzip
  echo "-- Extracting: boost_$VER_STR.tar.bz2"
  tar --bzip2 -xf "boost_$VER_STR.tar.bz2"
  cd boost_$VER_STR
else
  cd boost_$VER_STR
fi

# Bootstrap differently on Unix and Windows
if [ $OS_NAME == "Windows" ]; then
  ./bootstrap.bat
else
  ./bootstrap.sh
fi

# Build differently on QNX and non-QNX
if [ $OS_NAME == "QNX" ]; then
  if [ $QNX_ARCH == "x86_64" ]; then
    # Build for x86_64 target
    echo "Building boost for QNX x86_64"
    ./b2 -j$NUM_JOBS --prefix=$INSTALL_DIR --with-system variant=release link=shared toolset=qcc target-os=qnx architecture=x86 address-model=64 cxxflags="-Vgcc_ntox86_64" linkflags="-Vgcc_ntox86_64" install
  elif [ $QNX_ARCH == "aarch64" ]; then
    # Build for aarch64 target
    echo "Building boost for QNX aarch64"
    ./b2 -j$NUM_JOBS --prefix=$INSTALL_DIR --with-system variant=release link=shared toolset=qcc target-os=qnx architecture=arm address-model=64 cxxflags="-Vgcc_ntoaarch64le" linkflags="-Vgcc_ntoaarch64le" install
  else
    echo "Error: unknown QNX platform architecture"
    exit 1
  fi
else
  # Build for non-QNX
  ./b2 -j$NUM_JOBS --prefix=$INSTALL_DIR --with-system variant=release link=shared install
fi

echo "Installed boost"
