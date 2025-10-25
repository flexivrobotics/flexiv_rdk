cmake_minimum_required(VERSION 3.16.3)
project(flexiv_rdk-examples)

# Show verbose build info
SET(CMAKE_VERBOSE_MAKEFILE ON)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")

# Minimum example list for all OS
set(EXAMPLE_LIST
  basics1_display_robot_states
  basics2_clear_fault
  basics3_primitive_execution
  basics4_plan_execution
  basics5_zero_force_torque_sensors
  basics6_gripper_control
  basics7_auto_recovery
  basics8_update_robot_tool
  basics9_global_variables
  basics10_logging_behavior
  intermediate7_robot_dynamics
)

# Additional examples for Linux and Mac
if(CMAKE_HOST_UNIX)
  list(APPEND EXAMPLE_LIST
    intermediate1_realtime_joint_position_control
    intermediate2_realtime_joint_impedance_control
    intermediate3_realtime_joint_torque_control
    intermediate4_realtime_joint_floating
    intermediate5_realtime_cartesian_pure_motion_control
    intermediate6_realtime_cartesian_motion_force_control
  )
endif()

# Find flexiv_rdk INTERFACE library
find_package(flexiv_rdk REQUIRED)

# Build all selected examples
foreach(example ${EXAMPLE_LIST})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} flexiv::flexiv_rdk)

  # C++17 required
  set_target_properties(${example} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
endforeach()
