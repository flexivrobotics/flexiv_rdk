# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv RDK (Robotic Development Kit), a key component of the Flexiv Robotic Software Platform, is a powerful development toolkit that enables the users to create complex and customized robotic applications using APIs that provide both low-level real-time (RT) and high-level non-real-time (NRT) access to Flexiv robots.

## References

[Flexiv RDK Home Page](https://rdk.flexiv.com/) is the main reference. It contains important information including user manual and API documentation.

## Compatibility Overview

| **Supported OS**               | **Supported processor** | **Supported language** | **Required compiler kit** |
| ------------------------------ | ----------------------- | ---------------------- | ------------------------- |
| Linux (Ubuntu 18/20/22 tested) | x86_64, arm64           | C++, Python            | build-essential           |
| macOS 12 (Monterey)            | arm64                   | C++, Python            | Xcode Command Line Tools  |
| Windows 10                     | x86_64                  | C++, Python            | MSVC 14.0+                |

## Quick Start

**NOTE:** the full documentation is in [Flexiv RDK Manual](https://rdk.flexiv.com/manual/).

### C++ RDK

The C++ interface of Flexiv RDK is packed into a unified modern CMake library named ``flexiv_rdk``, which can be configured via CMake on all supported OS.

#### Compile and install for Linux

1. In a new Terminal, use ``cmake-gui`` to configure the top-level ``flexiv_rdk`` CMake project:

        cd flexiv_rdk
        mkdir build && cd build
        cmake-gui .. &

2. *Configure* with default native compiler settings.
3. Set ``CMAKE_INSTALL_PREFIX`` to a dedicated directory (preferably not a system path) for the ``flexiv_rdk`` library to be installed to. For example ``~/rdk_install``.
4. *Configure* and *Generate*, then back to the Terminal to compile and install:

        cd flexiv_rdk/build
        make install

5. The user project can now find and link to the installed ``flexiv_rdk`` library:

        cd flexiv_rdk/example
        mkdir build && cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=~/rdk_install
        make -j4

   Note: ``CMAKE_INSTALL_PREFIX`` is set to the same directory where ``flexiv_rdk`` was installed to.
6. Assuming the system setup detailed in the Flexiv RDK Manual is done, to run an compiled example program:

        ./<program_name> [robot_ip] [local_ip] [...]

     Note: ``sudo`` is not required unless prompted by the program.

#### Compile and install for Mac

1. In a Terminal, use ``xcode-select`` command to invoke the installation of Xcode Command Line Tools, then follow the prompted window to finish the installation.
2. Download ``cmake-3.x.x-macos-universal.dmg`` from [CMake download page](https://cmake.org/download/) and install the dmg file. The minimum required version is 3.4.
3. When done, start CMake from Launchpad and navigate to Tools -> How to Install For Command Line Use. Then follow the instruction "Or, to install symlinks to '/usr/local/bin', run:" to install ``cmake`` and ``cmake-gui`` command for use in Terminal.
4. The rest steps are the same as mentioned in [Compile and install for Linux](#compile-and-install-for-linux).

#### Compile and install for Windows

1. Install any edition of Microsoft Visual Studio with version 2010 or above. Choose the "Desktop development with C++" package during installation.
2. Download ``cmake-3.x.x-windows-x86_64.msi`` from [CMake download page](https://cmake.org/download/) and install the msi file. The minimum required version is 3.4. **Add CMake to system PATH** when prompted, so that ``cmake`` and ``cmake-gui`` command can be used from Command Prompt or PowerShell.
3. Configure the ``flexiv_rdk`` CMake project using the same steps mentioned in [Compile and install for Linux](#compile-and-install-for-linux).
4. Instead of ``make install``, use ``cmake`` to compile and install it:

        cmake --build . --target INSTALL --config Release

5. Configure the user project (RDK example programs) using the same steps mentioned in [Compile and install for Linux](#compile-and-install-for-linux).
6. Instead of ``make``, use ``cmake`` to compile it:

        cmake --build . --config Release

7. To run an compiled example program:

        cd Release
        <program_name>.exe [robot_ip] [local_ip] [...]

### Python RDK

Python 3.8 and 3.10 are supported by RDK, see Flexiv RDK Manual for more details. A brief instruction is provided below and applies to all supported OS.

1. Check Python version is 3.8.x or 3.10.x:

        python3 --version

2. Assume the system setup detailed in Flexiv RDK Manual is done, to run an example Python program:

        cd flexiv_rdk/example_py
        python3 <program_name>.py [robot_ip] [local_ip] [...]
