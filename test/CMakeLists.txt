cmake_minimum_required(VERSION 3.16.3)
project(flexiv_rdk-tests)

# Show verbose build info
SET(CMAKE_VERBOSE_MAKEFILE ON)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Tests for Windows
set(TEST_LIST
  test_log
)

# Additional tests for Linux and Mac
if(CMAKE_HOST_UNIX)
  list(APPEND TEST_LIST
    test_dynamics_engine
    test_dynamics_with_tool
    test_endurance
    test_loop_latency
    test_scheduler
    test_timeliness_monitor
  )
endif()

# Find flexiv_rdk INTERFACE library
find_package(flexiv_rdk REQUIRED)

# Build all selected tests
foreach(test ${TEST_LIST})
  add_executable(${test} ${test}.cpp)
  target_link_libraries(${test} flexiv::flexiv_rdk)
  # C++14 required
  set_target_properties(${test} PROPERTIES 
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON)
endforeach()
