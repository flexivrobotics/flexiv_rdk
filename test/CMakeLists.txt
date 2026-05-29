cmake_minimum_required(VERSION 3.22.1)
project(flexiv_rdk-tests)

# CMake >= 3.30 warns for FindBoost behavior unless CMP0167 is explicitly set.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

# Show verbose build info
SET(CMAKE_VERBOSE_MAKEFILE ON)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")

find_package(flexiv_rdk REQUIRED)

# List of functionality tests
set(FUNCTIONALITY_TESTS
  test_dynamics_engine
  test_repeated_instantiation
)
if(CMAKE_HOST_UNIX)
  list(APPEND FUNCTIONALITY_TESTS
    test_endurance
    test_loop_latency
    test_scheduler
    test_timeliness_monitor
  )
endif()

# Build functionality tests
foreach(test ${FUNCTIONALITY_TESTS})
  add_executable(${test} ${test}.cpp)
  target_link_libraries(${test} flexiv::flexiv_rdk)
  # C++20 required
  set_target_properties(${test} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON)
endforeach()


# List of unit tests
set(UNIT_TESTS
  test_robot_constructor
)

# Download GTest if not found
include(FetchContent)
find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
  if(MSVC)
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  endif()

  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  FetchContent_MakeAvailable(googletest)
endif()

# Build unit tests
foreach(test ${UNIT_TESTS})
  add_executable(${test} ${test}.cpp)
  target_link_libraries(${test} flexiv::flexiv_rdk GTest::gtest_main)
  add_test(NAME ${test} COMMAND ${test})
  # C++20 required
  set_target_properties(${test} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON)
endforeach()
