
cmake_minimum_required(VERSION 3.4)
project(flexiv_rdk-examples)

set(CMAKE_CXX_STANDARD 14)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Minumum example list for all OS
set(EXAMPLE_LIST
  auto_recovery
  clear_fault
  contact_control
  display_robot_states
  gripper_control
  plan_execution
  primitive_execution
  teach_by_demonstration
)


# Additional examples for Linux and Mac
if(CMAKE_HOST_UNIX)
  list(APPEND EXAMPLE_LIST
    RT_cartesian_impedance_control
    RT_joint_floating
    RT_joint_impedance_control
    RT_joint_position_control
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
