# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv RDK (Robotic Development Kit), a key component of the Flexiv Robotic Software Platform, is a powerful development toolkit that enables the users to create complex and customized robotic applications using APIs that provide both low-level real-time (RT) and high-level non-real-time (NRT) access to Flexiv robots.

## References

[Flexiv RDK Home Page](https://www.flexiv.com/software/rdk) is the main reference. It contains important information including user manual and API documentation. The instructions below serve as a quick reference, and you can find the full documentation at [Flexiv RDK Manual](https://www.flexiv.com/software/rdk/manual).

## Environment Compatibility

| **OS**                | **Platform**    | **C++ compiler kit** | **Python interpreter** |
| --------------------- | --------------- | -------------------- | ---------------------- |
| Linux (Ubuntu 20.04+) | x86_64, aarch64 | GCC   v9.4+          | 3.8, 3.10, 3.12        |
| macOS 12+             | arm64           | Clang v14.0+         | 3.10, 3.12             |
| Windows 10+           | x86_64          | MSVC  v14.2+         | 3.8, 3.10, 3.12        |
| QNX 8.0.2+            | x86_64, aarch64 | QCC   v12.2+         | Not supported          |

## Important Notice

Before trying to run any RDK program, please make sure to carefully go through the [First Time Setup](https://www.flexiv.com/software/rdk/manual/index.html#first-time-setup) chapter in RDK Manual. Otherwise, you might run into blocking issues.

## Quick Start - Python

### Install the Python package

On all supported platforms, the Python package of RDK and its dependencies for a specific Python version can be installed using the `pip` module:

    python3.x -m pip install numpy spdlog flexivrdk

NOTE: replace `3.x` with a specific Python version.

### Use the installed Python package

After the ``flexivrdk`` Python package is installed, it can be imported from any Python script. Test with the following commands in a new Terminal, which should start Flexiv RDK:

    python3.x
    import flexivrdk
    robot = flexivrdk.Robot("Rizon4-123456")

The program will start searching for a robot with serial number `Rizon4-123456`, and will exit after a couple of seconds if the specified robot is not found in the local network.

### Run example Python scripts

To run an example Python script in this repo:

    cd flexiv_rdk/example_py
    python3.x <example_name>.py [robot_serial_number]

For example:

    python3.10 ./basics1_display_robot_states.py Rizon4-123456

## Quick Start - C++

### Prepare build tools

#### Linux

1. Install compiler kit using package manager:

       sudo apt install build-essential

2. Install CMake using package manager:

       sudo apt install cmake

#### macOS

1. Install compiler kit using `xcode` tool:

       xcode-select 

   This will invoke the installation of Xcode Command Line Tools, then follow the prompted window to finish the installation.

2. Install CMake using package manager:

       brew install cmake

#### Windows

1. Install compiler kit: Download and install Microsoft Visual Studio 2019 (MSVC v14.2) or above. Choose "Desktop development with C++" under the *Workloads* tab during installation. You only need to keep the following components for the selected workload:
   * MSVC ... C++ x64/x86 build tools (Latest)
   * C++ CMake tools for Windows
   * Windows 10 SDK or Windows 11 SDK, depending on your actual Windows version
2. Install CMake: Download ``cmake-3.x.x-windows-x86_64.msi`` from [CMake download page](https://cmake.org/download/) and install the msi file. The minimum required version is 3.16.3. **Add CMake to system PATH** when prompted, so that ``cmake`` and ``cmake-gui`` command can be used from Command Prompt or a bash emulator.
3. Install bash emulator: Download and install [Git for Windows](https://git-scm.com/download/win/), which comes with a bash emulator Git Bash. The following steps are to be carried out in this bash emulator.

#### QNX

1. Prepare a host computer with Ubuntu 22.04 or higher.
2. Download and install [QNX SDP 8.0.3](https://blackberry.qnx.com/en/products/foundation-software/qnx-software-development-platform) to the host computer. You'll need a trial or commercial license.
3. Install CMake on the host computer using package manager:

       sudo apt install cmake

### Install the C++ library

The following steps are mostly the same on all supported platforms, with some variations.

1. Choose a directory for installing the C++ library of RDK and its dependencies. This directory can be under system path or not, depending on whether you want RDK to be globally discoverable by CMake. For example, a new folder named ``rdk_install`` under the home directory.
2. In a new Terminal, run the provided script to compile and install all dependencies to the installation directory chosen in step 1:

       cd flexiv_rdk/thirdparty

   For non-QNX:

       bash build_and_install_dependencies.sh ~/rdk_install

   For QNX:

       source <qnx-sdp-dir>/qnxsdp-env.sh
       bash build_and_install_dependencies.sh ~/rdk_install $(nproc) <path-to-qnx-toolchain-file>

   NOTE: the QNX toolchain files are located under ``flexiv_rdk/cmake`` directory, with one for x86_64 target and one for aarch64 target.

3. In the same Terminal, configure the ``flexiv_rdk`` CMake project:

       cd flexiv_rdk
       mkdir build && cd build

   For non-QNX:

       cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install

   For QNX:

       cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install -DCMAKE_TOOLCHAIN_FILE=<path-to-qnx-toolchain-file>

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` sets the absolute path of the installation directory, which should be the one chosen in step 1.

4. Install ``flexiv_rdk`` C++ library to ``CMAKE_INSTALL_PREFIX`` path, which may or may not be globally discoverable by CMake depending on the location:

       cd flexiv_rdk/build
       cmake --build . --target install --config Release

### Use the installed C++ library

After the library is installed as ``flexiv_rdk`` CMake target, it can be linked from any other CMake projects. Using the provided `flexiv_rdk-examples` project for instance:

    cd flexiv_rdk/example
    mkdir build && cd build

For non-QNX:

    cmake .. -DCMAKE_PREFIX_PATH=~/rdk_install
    cmake --build . --config Release -j 4

For QNX:

    cmake .. -DCMAKE_PREFIX_PATH=~/rdk_install -DCMAKE_TOOLCHAIN_FILE=<path-to-qnx-toolchain-file>
    cmake --build . --config Release -j 4

NOTE: ``-D`` followed by ``CMAKE_PREFIX_PATH`` tells the user project's CMake where to find the installed C++ library. This argument can be skipped if the RDK library and its dependencies are installed to a globally discoverable location.

### Run example C++ programs

To run an example C++ program compiled during the previous step:

    cd flexiv_rdk/example/build
    ./<example_name> [robot_serial_number]

For example:

    ./basics1_display_robot_states Rizon4-123456

NOTE: ``sudo`` is only required if the real-time scheduler API ``flexiv::rdk::Scheduler`` is used in the program.

## API Documentation

The complete and detailed API documentation of the **latest release** can be found at https://www.flexiv.com/software/rdk/api. The API documentation of a previous release can be generated manually using Doxygen. For example, on Linux:

    sudo apt install doxygen-latex graphviz
    cd flexiv_rdk
    git checkout <previous_release_tag>
    doxygen doc/Doxyfile.in

Open any html file under ``flexiv_rdk/doc/html/`` with your browser to view the doc.
