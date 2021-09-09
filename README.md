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

1. Configure CMake:

        cd flexiv_rdk
        mkdir build && cd build
        cmake ..

2. Update ``LOCAL_IP`` in ``include/config.h`` with the actual IP address of your workstation PC in the same subnet with the robot.
3. Build example programs:

        cd flexiv_rdk/build
        make

4. The compiled program binaries will be output to ``flexiv_rdk/build/example``
5. Assuming the robot is booted and connected, to run an example program:

        cd flexiv_rdk/build/example
        sudo ./<program_name>
