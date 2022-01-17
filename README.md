# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Flexiv RDK (Robot Development Kit) is a powerful toolkit as an add-on to the Flexiv software platform. It enables the users to create complex applications with APIs that provide low-level real-time access to the robot.

**C++** interface (real-time and non-real-time APIs) and **Python** interface (non-real-time APIs only) are supported.

## License

Flexiv RDK is licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).

## References

**[Flexiv RDK Manual](https://flexivrobotics.github.io/)** is the main reference and contains important information on how to properly set up the system and use Flexiv RDK library.

[API documentation](https://flexivrobotics.github.io/flexiv_rdk/) contains details about available APIs, generated from Doxygen.

## Run example programs

**NOTE:** the instruction below is only a quick reference, assuming you've already gone through the Flexiv RDK Manual.

### C++ interface

1. Configure and build example programs:

        cd flexiv_rdk
        mkdir build && cd build
        cmake ..
        make -j4

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
