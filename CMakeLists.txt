cmake_minimum_required(VERSION 3.16.3)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_rdk VERSION 1.8.0)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")

# Set static library according to platform
message(STATUS "OS: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(RDK_LIB "libflexiv_rdk.x86_64-linux-gnu.a")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(RDK_LIB "libflexiv_rdk.aarch64-linux-gnu.a")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64")
    set(RDK_LIB "libflexiv_rdk.arm64-darwin.a")
  else()
    message(FATAL_ERROR "macOS with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "AMD64")
    set(RDK_LIB "flexiv_rdk.win_amd64.lib")
  else()
    message(FATAL_ERROR "Windows with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "QNX")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(RDK_LIB "libflexiv_rdk.x86_64-qnx.a")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64le")
    set(RDK_LIB "libflexiv_rdk.aarch64-qnx.a")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
endif()

# Option to support ROS2 Jazzy
option(RDK_SUPPORT_ROS2_JAZZY "Use RDK library compatible with ROS2 Jazzy" OFF)
if(RDK_SUPPORT_ROS2_JAZZY)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Linux" AND ${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(RDK_LIB "libflexiv_rdk.x86_64-linux-gnu.ros2-jazzy.a")
  else()
    message(FATAL_ERROR "ROS2 Jazzy compatibility is currently only supported on x86_64 Linux")
  endif()
endif()

# Parse expected hash of the library file
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/ref_hash/${RDK_LIB}.sha256 EXPECTED_HASH)

# Download the library file from GitHub 
set(RDK_LIB_URL https://github.com/flexivrobotics/flexiv_rdk/releases/download/test_release/${RDK_LIB})
message(STATUS "Downloading ${RDK_LIB_URL} ...")
file(DOWNLOAD ${RDK_LIB_URL} ${CMAKE_CURRENT_SOURCE_DIR}/lib/${RDK_LIB}
     SHOW_PROGRESS
     TLS_VERIFY ON
     EXPECTED_HASH SHA256=${EXPECTED_HASH}
     STATUS status)
list(GET status 0 code)
if(code EQUAL 0)
  message(STATUS "Download finished")
else()
  message(FATAL_ERROR "Download failed: ${status}")
endif()

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================
# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
if(Threads_FOUND)
  message(STATUS "Found Threads")
endif()

# Eigen3
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  message(STATUS "Found Eigen3: ${Eigen3_DIR}")
endif()

# spdlog
find_package(spdlog REQUIRED)
if(spdlog_FOUND)
  message(STATUS "Found spdlog: ${spdlog_DIR}")
endif()

# Fast-DDS (Fast-RTPS)
find_package(fastrtps 2.6.7 REQUIRED)
if(fastrtps_FOUND)
  message(STATUS "Found fastrtps: ${fastrtps_DIR}")
endif()

# ===================================================================
#      CREATE LIBRARY
# ===================================================================
# Create an INTERFACE library with no source file to compile
add_library(${PROJECT_NAME} INTERFACE)

# Create an alias of the library using flexiv namespace,
# to imitate the install target which uses flexiv namespace.
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${RDK_LIB}
  Threads::Threads
  Eigen3::Eigen
  spdlog::spdlog
  fastrtps
)

# Use moderate compiler warning option
if(CMAKE_HOST_UNIX)
  target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra)
else()
  target_compile_options(${PROJECT_NAME} INTERFACE /W1)
endif()

# Install the INTERFACE library
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()
