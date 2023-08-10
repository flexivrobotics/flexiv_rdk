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
    echo "    num_parallel_jobs: number of parallel jobs used to build, if not specified, the number of CPU cores will be used"
    exit
fi

# Get dependencies install directory from script argument, should be the same as the install directory of flexiv_rdk
INSTALL_DIR=$1
echo "Dependencies will be installed to: $INSTALL_DIR"

# Use specified number for parallel build jobs, otherwise use number of cores 
if [ -n "$2" ] ;then
    NUM_JOBS=$2
else
    NUM_JOBS=$(nproc)
fi
echo "Number of parallel build jobs: $NUM_JOBS"


# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned

# Build and install all dependencies to INSTALL_DIR
bash $SCRIPTPATH/scripts/install_eigen.sh $INSTALL_DIR $NUM_JOBS
bash $SCRIPTPATH/scripts/install_tinyxml2.sh $INSTALL_DIR $NUM_JOBS
bash $SCRIPTPATH/scripts/install_foonathan_memory.sh $INSTALL_DIR $NUM_JOBS
bash $SCRIPTPATH/scripts/install_Fast-CDR.sh $INSTALL_DIR $NUM_JOBS
bash $SCRIPTPATH/scripts/install_Fast-DDS.sh $INSTALL_DIR $NUM_JOBS

echo ">>>>>>>>>> Finished <<<<<<<<<<"
