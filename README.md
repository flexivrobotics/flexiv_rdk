# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv RDK (Robotic Development Kit), a key component of the Flexiv Robotic Software Platform, is a powerful development toolkit that enables the users to create complex and customized robotic applications using APIs that provide both low-level real-time (RT) and high-level non-real-time (NRT) access to Flexiv robots.

## References

[Flexiv RDK Home Page](https://rdk.flexiv.com/) is the main reference. It contains important information including user manual and API documentation.

## Compatibility Overview

| **Supported OS**           | **Supported processor** | **Supported language** | **Required compiler kit** |
| -------------------------- | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu 20.04/22.04) | x86_64, arm64           | C++, Python            | build-essential           |
| macOS 12 and above         | arm64                   | C++, Python            | Xcode Command Line Tools  |
| Windows 10/11              | x86_64                  | C++, Python            | MSVC v14.2+               |

## Quick Start

The **C++ and Python** RDK libraries are packed into a unified modern CMake project named ``flexiv_rdk``, which can be configured and installed via CMake on all supported OS.

### Note

* The instructions below serve as a quick reference. You can find the full documentation at [Flexiv RDK Manual](https://rdk.flexiv.com/manual/).
* You might need to turn off your computer's firewall or whitelist the RDK programs to be able to establish connection with the robot.

### Install on Linux

1. In a new Terminal, install compiler kit, CMake (with GUI), Python interpreter, and Python package manager:

       sudo apt install build-essential cmake cmake-qt-gui python3 python3-pip -y

2. Choose a directory for installing C++ RDK library and all its dependencies. This directory can be under system path or not, depending on whether you want RDK to be globally discoverable by CMake. For example, a new folder named ``rdk_install`` under the home directory.
3. In a new Terminal, run the provided script to compile and install all C++ dependencies to the installation directory chosen in step 2:

       cd flexiv_rdk/thirdparty
       bash build_and_install_dependencies.sh ~/rdk_install

4. In a new Terminal, use CMake to configure the ``flexiv_rdk`` project:

       cd flexiv_rdk
       mkdir build && cd build
       cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install -DINSTALL_PYTHON_RDK=ON

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` sets the CMake variable that specifies the path of the installation directory. ``-D`` followed by ``INSTALL_PYTHON_RDK=ON`` enables the installation of Python RDK besides C++ RDK. The configuration process can also be done using CMake GUI.

5. Install C++ and Python RDK:

       cd flexiv_rdk/build
       cmake --build . --target install --config Release

   C++ RDK is installed to the path specified by ``CMAKE_INSTALL_PREFIX``, which may or may not be globally discoverable by CMake. Python RDK is installed to the user site packages path, which is globally discoverable by Python interpreter.

### Install on macOS

1. Install compiler kit: In a new Terminal, enter command ``xcode-select`` to invoke the installation of Xcode Command Line Tools, then follow the prompted window to finish the installation.
2. Install CMake (with GUI): Download ``cmake-3.x.x-macos-universal.dmg`` from [CMake download page](https://cmake.org/download/) and install the dmg file. The minimum required version is 3.16.3. When done, start CMake from Launchpad and navigate to Tools -> How to Install For Command Line Use. Then follow the instruction "Or, to install symlinks to '/usr/local/bin', run:" to install ``cmake`` and ``cmake-gui`` commands for use in Terminal.
3. Install Python interpreter and package manager (replace "3.x" with the actual Python3 version you wish to use):

       brew install python@3.x

4. The rest are identical to steps 2 and below in [Install on Linux](#install-on-linux).

### Install on Windows

1. Install compiler kit: Download and install Microsoft Visual Studio 2019 (MSVC v14.2) or above. Choose "Desktop development with C++" under the *Workloads* tab during installation. You only need to keep the following components for the selected workload:
   * MSVC ... C++ x64/x86 build tools (Latest)
   * C++ CMake tools for Windows
   * Windows 10 SDK or Windows 11 SDK, depending on your actual Windows version
2. Install CMake (with GUI): Download ``cmake-3.x.x-windows-x86_64.msi`` from [CMake download page](https://cmake.org/download/) and install the msi file. The minimum required version is 3.16.3. **Add CMake to system PATH** when prompted, so that ``cmake`` and ``cmake-gui`` command can be used from Command Prompt or a bash emulator.
3. Install bash emulator: Download and install [Git for Windows](https://git-scm.com/download/win/), which comes with a bash emulator Git Bash.
4. Within the bash emulator, the rest are identical to steps 2 and below in [Install on Linux](#install-on-linux).

### Link to C++ RDK from a user program

After C++ RDK is installed, it can be found as a CMake library and linked to by other CMake projects. Use the provided examples project for instance::

    cd flexiv_rdk/example
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install
    cmake --build . --config Release -j 4

NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` tells the user project's CMake where to find the installed C++ RDK library. The instruction above applies to all supported OS.

### Run example programs

#### C++

To run a compiled example C++ program:

    cd flexiv_rdk/example/build
    ./<program_name> [robot_serial_number]

For example:

    ./basics1_display_robot_states Rizon4s-123456

NOTE: ``sudo`` is only required if the real-time scheduler API ``flexiv::rdk::Scheduler`` is used.

#### Python

To run a example Python program:

    cd flexiv_rdk/example_py
    python3 <program_name>.py [robot_serial_number]

For example:

    python3 ./basics1_display_robot_states.py Rizon4s-123456

## API Documentation

The complete and detailed API documentation of the **latest release** can be found at https://rdk.flexiv.com/api/. The API documentation of a previous release can be generated manually using Doxygen. For example, on Linux:

    sudo apt install doxygen-latex graphviz
    cd flexiv_rdk
    git checkout <previous_release_tag>
    doxygen doc/Doxyfile.in

The generated API documentation is under ``flexiv_rdk/doc/html/`` directory. Open any html file with your browser to view it.
