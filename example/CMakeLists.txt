
cmake_minimum_required(VERSION 3.4)
project(flexiv_rdk-examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON)
message("Building for system: ${CMAKE_SYSTEM_NAME}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Minumum example list for Windows 
set(EXAMPLE_LIST
  auto_recovery
  clear_fault
  display_robot_states
  gripper_control
  plan_execution
  primitive_execution
  teach_by_demonstration
)

# Additional examples for Mac
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(EXAMPLE_LIST ${EXAMPLE_LIST}
    robot_dynamics
  )
endif()

# Additional examples for Linux only
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(EXAMPLE_LIST ${EXAMPLE_LIST}
    cartesian_impedance_control
    floating_with_soft_limits
    joint_impedance_control
    joint_position_control
    robot_dynamics
    series_operation
  )
endif()

# Find flexiv_rdk INTERFACE library
find_package(flexiv_rdk REQUIRED)

# Build all selected examples
foreach(example ${EXAMPLE_LIST})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} flexiv::flexiv_rdk)
endforeach()
