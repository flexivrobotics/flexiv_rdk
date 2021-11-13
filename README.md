# Flexiv RDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_rdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Flexiv Robot Development Kit (RDK) is a C++ library for creating powerful real-time applications with Flexiv robots.

## License

Flexiv RDK is licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).

## References

Please refer to **[Flexiv RDK Manual](https://flexivrobotics.github.io/)** on how to properly set up the system and use Flexiv RDK library.

For details about the APIs, please refer to the [Doxygen generated API documentation](https://flexivrobotics.github.io/flexiv_rdk/).

## Build and run the examples

**NOTE:** the following is only a quick reference, assuming you've already gone through Flexiv RDK Manual.

1. Configure and build example programs:

        cd flexiv_rdk
        mkdir build && cd build
        cmake ..
        make -j4

2. The compiled program binaries will be output to ``flexiv_rdk/build/example``
3. Assuming the robot is booted and connected, to run an example program:

        cd flexiv_rdk/build/example
        sudo ./<program_name> <robot_ip> <local_ip>

## Integrated visualization

Flexiv RDK has integrated visualization based on Meshcat. To use it, follow instructions below.

1. Install Meshcat server using ``pip``:

        pip install meshcat

2. Start the Meshcat server in Terminal:

        meshcat-server

3. It will provide several URLs, open ``web_url`` with a web browser. An empty scene will open up.

4. Use APIs in ``flexiv::Visualization`` to communicate with the Meshcat server, the empty scene will be populated with robot(s) and user-defined objects if any. Refer to ``example/visualization.cpp``.

**Note:** only STL mesh files are supported, which are provided in ``spec/meshes``.
