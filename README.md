# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Flexiv RDK (Robot Development Kit) is a powerful toolkit as an add-on to the Flexiv software platform. It enables the users to create complex applications with APIs that provide low-level real-time access to the robot.

**C++** (access to all APIs) and **Python** (access to non-real-time APIs only) are supported.

## License

Flexiv RDK is licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).

## References

[Flexiv RDK main webpage](https://rdk.flexiv.com/) contains important information like the user manual and API documentation, please read them carefully before proceeding.

## Run example programs

**NOTE:** the instruction below is only a quick reference, assuming you've already gone through the Flexiv RDK Manual.

### C++ interface

1. Configure and compile example programs for x64 processors:

        cd flexiv_rdk
        mkdir build && cd build
        cmake ..
        make -j4

   For arm64 processors, set the additional CMake option when configuring:

        cmake .. -DBUILD_FOR_ARM64=ON

2. The compiled program binaries will be output to ``flexiv_rdk/build/example``
3. Assume the robot is booted and connected, to run an example program:

        cd flexiv_rdk/build/example
        sudo ./<program_name> <robot_ip> <local_ip> <...>

### Python interface

**NOTE:** Python 3.8 is required to use this interface, see Flexiv RDK Manual for more details.

1. Make sure your Python version is 3.8.x:

        python3 --version

2. Assume the robot is booted and connected, to run an example program:

        cd flexiv_rdk/example_py
        sudo python3 <program_name>.py <robot_ip> <local_ip> <...>

## Integrated visualization

Flexiv RDK has integrated visualization (only available for C++ interface) based on Meshcat. To use this feature:

1. Install Meshcat server using ``pip``:

        pip install meshcat

2. Start the Meshcat server in Terminal:

        meshcat-server

3. It will provide several URLs, open ``web_url`` with a web browser. An empty scene will open up.

4. Use APIs in ``flexiv::Visualization`` to communicate with the Meshcat server, the empty scene will be populated with robot(s) and user-defined objects if any. Refer to ``example/visualization.cpp``.

**Note:** only STL mesh files are supported, which are provided in ``spec/meshes``.

