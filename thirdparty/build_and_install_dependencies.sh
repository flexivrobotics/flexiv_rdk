#!/bin/sh
# This script builds from source and installs all dependencies of flexiv_rdk.

# Absolute path of this script
SCRIPTPATH="$(dirname $(readlink -f $0))"
set -e

# Check script arguments
if [ "$#" -lt 1 ]; then
    echo "Error: invalid script argument"
    echo "Required argument: [install_directory_path]"
    echo "    install_directory_path: directory to install all dependencies, should be the same as the install directory of flexiv_rdk"
    echo "Optional argument: [num_parallel_jobs]"
    echo "    num_parallel_jobs: number of parallel jobs used to build, use 4 if not specified"
    exit
fi

# Get dependencies install directory from script argument, should be the same as the install directory of flexiv_rdk
INSTALL_DIR=$1
echo "Dependencies will be installed to: $INSTALL_DIR"

# Use specified number for parallel build jobs, otherwise use number of cores 
if [ -n "$2" ] ;then
    NUM_JOBS=$2
else
    NUM_JOBS=4
fi
echo "Number of parallel build jobs: $NUM_JOBS"


# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned

# Dependencies to install via package manager or build from source based on OS
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    SUDO="sudo"
    if [ -w /usr/lib/ ]; then SUDO=""; fi
    $SUDO apt-get install -y libeigen3-dev libspdlog-dev libtinyxml2-dev
elif [[ "$OSTYPE" == "darwin"* ]]; then
    brew install eigen spdlog tinyxml2
else
    bash $SCRIPTPATH/scripts/install_eigen.sh $INSTALL_DIR $NUM_JOBS
    bash $SCRIPTPATH/scripts/install_spdlog.sh $INSTALL_DIR $NUM_JOBS
    bash $SCRIPTPATH/scripts/install_tinyxml2.sh $INSTALL_DIR $NUM_JOBS
fi

# Dependencies to build from source regardless of OS
bash $SCRIPTPATH/scripts/install_foonathan_memory.sh $INSTALL_DIR $NUM_JOBS
bash $SCRIPTPATH/scripts/install_Fast-CDR.sh $INSTALL_DIR $NUM_JOBS
bash $SCRIPTPATH/scripts/install_Fast-DDS.sh $INSTALL_DIR $NUM_JOBS

echo ">>>>>>>>>> Finished <<<<<<<<<<"
