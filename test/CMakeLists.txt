
cmake_minimum_required(VERSION 3.4)
project(flexiv_rdk-tests)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_VERBOSE_MAKEFILE ON)
message("Building for system: ${CMAKE_SYSTEM_NAME}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Tests for Mac/Windows
set(TEST_LIST
  test_log
)

# Additional tests for Linux only
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(TEST_LIST ${TEST_LIST}
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

# Build all selected examples
foreach(test ${TEST_LIST})
  add_executable(${test} ${test}.cpp)
  target_link_libraries(${test} flexiv::flexiv_rdk)
endforeach()
