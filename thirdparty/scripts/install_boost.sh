#!/usr/bin/env bash
set -e
repo="boost"
echo "Installing $repo"

# Function to check if running on Windows
is_windows() {
  [[ "$OSTYPE" == "msys"* || "$OSTYPE" == "cygwin"* || "$OSTYPE" == "mingw"* || "$OSTYPE" == "win32"* ]]
}

# Default version on Ubuntu 24.04
# Also the minimum version compatible with MSVC v143
ver_tag=1.83.0
ver_str=1_83_0

# Download source code, 1.83 
if [ ! -d ${repo}_${ver_str} ] ; then
  # download is faster than clone
  URL="https://archives.boost.io/release/$ver_tag/source/${repo}_${ver_str}.tar.bz2"
  echo "-- Downloading: $URL"
  if is_windows; then
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
if is_windows; then
  # Windows
  ./bootstrap.bat
else
  # Unix
  ./bootstrap.sh
fi

./b2 -j$NUM_JOBS --prefix=$INSTALL_DIR --with-system --with-filesystem variant=release link=shared install

echo "Installed $repo"
