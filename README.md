# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Flexiv RDK (Robotic Development Kit), a key component of the Flexiv Robotic Software Platform, is a powerful development toolkit that enables the users to create complex and customized robotic applications using APIs that provide both low-level real-time (RT) and high-level non-real-time (NRT) access to Flexiv robots.

## License

Flexiv RDK is licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).

## References

[Flexiv RDK main webpage](https://rdk.flexiv.com/) contains important information like the user manual and API documentation, please read them carefully before proceeding.

## OS and language support

Supported OS:

* Linux: Ubuntu 18.04 / 20.04
* Windows: 10 with MSVC 14.0+

Supported programming languages:

* C++ (Linux and Windows)
* Python (Linux only)

## Run example programs

**NOTE:** the instruction below is only a quick reference, assuming you've already gone through [Flexiv RDK Manual](https://rdk.flexiv.com/manual/).

### Linux C++ interface

1. Configure CMake and compile all C++ example programs for the x64 processor platform:

        cd flexiv_rdk
        mkdir build && cd build
        cmake ..
        make -j4

   If compiling for the arm64 processor platform, set the additional CMake option when configuring:

        cmake .. -DBUILD_FOR_ARM64=ON

2. The compiled program binaries will be output to ``flexiv_rdk/build/example``
3. Assume the robot is booted and connected, to run an example program from a Linux Terminal:

        cd flexiv_rdk/build/example
        ./<program_name> <robot_ip> <local_ip> <...>

   If ``flexiv::Scheduler`` is used in this program, then ``sudo`` is required to grant root privileges to the integrated scheduler:

        sudo ./<program_name> <robot_ip> <local_ip> <...>

### Linux Python interface

**NOTE:** Python 3.8 is required to use this interface, see Flexiv RDK Manual for more details.

1. Make sure your Python version is 3.8.x:

        python3 --version

2. Assume the robot is booted and connected, to run an example program from a Linux Terminal:

        cd flexiv_rdk/example_py
        python3 <program_name>.py <robot_ip> <local_ip> <...>

   Note that ``sudo`` is not required for Python interface.

### Windows C++ interface

1. Install any edition of Microsoft Visual Studio with version 2010 or above.
2. Use Visual Studio to open the example solution file at ``flexiv_rdk\example\windows\windows.sln``. Several example projects are pre-configured for x64 and x86 processor platforms, with each of them corresponding to an example under ``flexiv_rdk\example\``.
3. Check that the global solution configuration is set to **Release**. This setting can usually be found near the top menu bar.
4. Use *Build* -> *Build Solution* to compile all example projects. The compiled ``.exe`` executable binary files are output to ``flexiv_rdk\build\windows\``.
5. Assume the robot is booted and connected, to run an example program from a Windows Command Prompt:

        cd flexiv_rdk_dev\build\windows\<platform>\Release\
        <program_name>.exe <robot_ip> <local_ip> <...>

## Integrated visualization

Flexiv RDK has integrated visualization (only available for **Linux C++ interface**) based on Meshcat. To use this feature:

1. Install Meshcat server using ``pip``:

        pip install meshcat

2. Start the Meshcat server in Terminal:

        meshcat-server

3. It will provide several URLs, open ``web_url`` with a web browser. An empty scene will open up.

4. Use APIs in ``flexiv::Visualization`` to communicate with the Meshcat server, the empty scene will be populated with robot(s) and user-defined objects if any. Refer to ``example/visualization.cpp``.

**Note:** only STL mesh files are supported, which are provided in ``spec/meshes``.

