#!/bin/sh
# This script builds from source and installs all dependencies of flexiv_rdk.

# Absolute path of this script
export SCRIPTPATH="$(dirname $(readlink -f $0))"
set -e

# Check script arguments
if [ "$#" -lt 1 ]; then
    echo "Error: invalid script argument"
    echo "Required argument: [install_directory_path]"
    echo "    install_directory_path: directory to install all dependencies, should be the same as the install directory of flexiv_rdk"
    echo "Optional arguments: [num_parallel_jobs] [qnx_toolchain_path]"
    echo "    num_parallel_jobs: number of parallel jobs used to build, use 4 if not specified"
    echo "    qnx_toolchain_path: path to the QNX toolchain file under cmake/qnx/, required if building for QNX"
    exit
fi

# Get dependencies install directory from script argument, should be the same as the install directory of flexiv_rdk
INSTALL_DIR=$1
echo "Dependencies will be installed to: $INSTALL_DIR"

# Use specified number for parallel build jobs, otherwise use 4
if [ -n "$2" ] ;then
    export NUM_JOBS=$2
else
    export NUM_JOBS=4
fi
echo "Number of parallel build jobs: $NUM_JOBS"

# Set shared cmake arguments
SHARED_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release \
                   -DBUILD_SHARED_LIBS=OFF \
                   -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
                   -DCMAKE_PREFIX_PATH=$INSTALL_DIR \
                   -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
                   -DBUILD_TESTING=OFF"

# Building for QNX
if [ -n "$QNX_TARGET" ]; then
    # Path to the toolchain file must be set
    if [ -n "$3" ]; then
        QNX_TOOLCHAIN=$3
    else
        echo "Error: path to QNX toolchain file is not provided"
        exit 1
    fi
    # Append toolchain file to cmake arguments
    SHARED_CMAKE_ARGS="$SHARED_CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=$QNX_TOOLCHAIN"
    echo "Building for QNX with toolchain [$QNX_TOOLCHAIN]"
fi

# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned
export SHARED_CMAKE_ARGS

# Build and install all dependencies to INSTALL_DIR
bash $SCRIPTPATH/scripts/install_eigen.sh
bash $SCRIPTPATH/scripts/install_spdlog.sh
bash $SCRIPTPATH/scripts/install_tinyxml2.sh
bash $SCRIPTPATH/scripts/install_foonathan_memory.sh
bash $SCRIPTPATH/scripts/install_Fast-CDR.sh
bash $SCRIPTPATH/scripts/install_Fast-DDS.sh

echo ">>>>>>>>>> Finished <<<<<<<<<<"
