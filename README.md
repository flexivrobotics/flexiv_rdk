# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv RDK (Robotic Development Kit), a key component of the Flexiv Robotic Software Platform, is a powerful development toolkit that enables the users to create complex and customized robotic applications using APIs that provide both low-level real-time (RT) and high-level non-real-time (NRT) access to Flexiv robots.

## References

[Flexiv RDK Home Page](https://www.flexiv.com/software/rdk) is the main reference. It contains important information including user manual and API documentation. The instructions below serve as a quick reference, and you can find the full documentation at [Flexiv RDK Manual](https://www.flexiv.com/software/rdk/manual).

## Compatibility Overview

| **OS**                | **Platform**  | **C++ compiler kit** | **Python version** |
| --------------------- | ------------- | -------------------- | ------------------ |
| Linux (Ubuntu 20.04+) | x86_64, arm64 | GCC   v9.4+          | 3.8, 3.10, 3.12    |
| macOS 12+             | arm64         | Clang v14.0+         | 3.10, 3.12         |
| Windows 10+           | x86_64        | MSVC  v14.2+         | 3.8, 3.10, 3.12    |

**IMPORTANT**: You might need to turn off your computer's firewall or whitelist the RDK programs to be able to establish connection with the robot.

## Quick Start - Python

### Install RDK Python library

For all supported platforms, the RDK Python library and its dependencies for a specific version of Python can be installed using its `pip` module:

    python3.x -m pip install numpy spdlog flexivrdk

Replace `3.x` with a specific Python version.

### Use the installed Python library

After the library is installed as ``flexivrdk`` Python package, it can be imported from any Python script. Test with the following commands in a new Terminal, which should start Flexiv RDK:

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

### Install RDK C++ library

The following steps are identical on all supported platforms.

1. Choose a directory for installing RDK C++ library and all its dependencies. This directory can be under system path or not, depending on whether you want RDK to be globally discoverable by CMake. For example, a new folder named ``rdk_install`` under the home directory.
2. In a new Terminal, run the provided script to compile and install all C++ dependencies to the installation directory chosen in step 1:

       cd flexiv_rdk/thirdparty
       bash build_and_install_dependencies.sh ~/rdk_install

3. In a new Terminal, configure the ``flexiv_rdk`` CMake project:

       cd flexiv_rdk
       mkdir build && cd build
       cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` sets the absolute path of the installation directory, which should be the one chosen in step 1.

4. Install ``flexiv_rdk`` C++ library to ``CMAKE_INSTALL_PREFIX`` path, which may or may not be globally discoverable by CMake:

       cd flexiv_rdk/build
       cmake --build . --target install --config Release

### Use the installed C++ library

After the library is installed as ``flexiv_rdk`` CMake target, it can be linked from any other CMake projects. Using the provided `flexiv_rdk-examples` project for instance:

    cd flexiv_rdk/example
    mkdir build && cd build
    cmake .. -DCMAKE_PREFIX_PATH=~/rdk_install
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

The generated API documentation is under ``flexiv_rdk/doc/html/`` directory. Open any html file with your browser to view it.
