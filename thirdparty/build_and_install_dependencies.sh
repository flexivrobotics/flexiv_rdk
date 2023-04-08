#!/bin/sh
# This script builds from source and installs all dependencies of flexiv_rdk.

# Absolute path of this script
SCRIPTPATH="$(dirname $(readlink -f $0))"
set -e

# Get dependencies install directory from script argument, 
# should be the same as the install directory of flexiv_rdk
if [ "$#" -ne 1 ]; then
    echo "Error: path to install directory is not provided"
    echo "Usage: bash build_and_install_dependencies.sh [install_directory_path]"
    exit
fi
INSTALL_DIR=$1
echo "Dependencies will be installed to: $INSTALL_DIR"

# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned

# Build and install all dependencies to INSTALL_DIR
bash $SCRIPTPATH/scripts/install_eigen.sh $INSTALL_DIR
bash $SCRIPTPATH/scripts/install_tinyxml2.sh $INSTALL_DIR
bash $SCRIPTPATH/scripts/install_foonathan_memory.sh $INSTALL_DIR
bash $SCRIPTPATH/scripts/install_Fast-CDR.sh $INSTALL_DIR
bash $SCRIPTPATH/scripts/install_Fast-DDS.sh $INSTALL_DIR

echo ">>>>>>>>>> Finished <<<<<<<<<<"
