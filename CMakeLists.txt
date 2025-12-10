cmake_minimum_required(VERSION 3.16.3)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_rdk VERSION 1.9.0)

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
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${RDK_LIB}.sha256 EXPECTED_HASH)

# Download the library file from GitHub 
set(RDK_LIB_URL https://github.com/flexivrobotics/flexiv_rdk/releases/download/v1.9_dev/${RDK_LIB})
message(STATUS "Downloading ${RDK_LIB_URL} ...")
file(DOWNLOAD ${RDK_LIB_URL} ${CMAKE_CURRENT_BINARY_DIR}/${RDK_LIB}
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
message(STATUS "Found Threads")

# Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3 v${Eigen3_VERSION}: ${Eigen3_DIR}")

# spdlog
find_package(spdlog REQUIRED)
message(STATUS "Found spdlog v${spdlog_VERSION}: ${spdlog_DIR}")

# Fast-DDS (Fast-RTPS)
find_package(fastrtps REQUIRED)
message(STATUS "Found fastrtps v${fastrtps_VERSION}: ${fastrtps_DIR}")

# Fast-CDR
find_package(fastcdr REQUIRED)
message(STATUS "Found fastcdr v${fastcdr_VERSION}: ${fastcdr_DIR}")

# RBDyn
find_package(RBDyn REQUIRED)
message(STATUS "Found RBDyn v${RBDyn_VERSION}: ${RBDyn_DIR}")

# ===================================================================
#      CREATE LIBRARY
# ===================================================================
# Create static library target with dummy source
add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/dummy.cpp)

# Create an alias of the target with flexiv namespace
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  Threads::Threads
  Eigen3::Eigen
  spdlog::spdlog
  fastrtps
  fastcdr
  RBDyn::RBDyn
  RBDyn::Parsers
)

# Use moderate compiler warning option
if(CMAKE_HOST_UNIX)
  target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)
else()
  target_compile_options(${PROJECT_NAME} PUBLIC /W1)
endif()

# Install the library
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()
