#! /bin/sh 
# =========================================================================
# NOTE: 
# These dependencies are only required by the user code in some examples, 
# and are NOT required by the RDK library itself. Please refer to Flexiv 
# RDK documentation/Getting Started/Build and run examples for more details
# =========================================================================

# Dependencies for keyboad_move_tcp.cpp
# -----------------------------------------------------------------------
# install curl
sudo apt install curl

# install RBDyn(C++)
curl -1sLf \
  'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' \
  | sudo -E bash

sudo apt install librbdyn-dev

# install boost
sudo apt install libboost-all-dev
# -----------------------------------------------------------------------
