cmake_minimum_required(VERSION 3.22.1)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_rdk VERSION 2.1 LANGUAGES CXX)

# CMake >= 3.30 warns for FindBoost behavior unless CMP0167 is explicitly set.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

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
    message(FATAL_ERROR "QNX with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
else()
  message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} is currently not supported.")
endif()

# Parse expected hash of the library file
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${RDK_LIB}.sha256 EXPECTED_HASH)

# Download the library file from GitHub 
set(RDK_LIB_URL https://github.com/flexivrobotics/flexiv_rdk/releases/download/v${PROJECT_VERSION}/${RDK_LIB})
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
find_package(Eigen3 3.4.0 REQUIRED)
message(STATUS "Found Eigen3 v${Eigen3_VERSION}: ${Eigen3_DIR}")

# spdlog
find_package(spdlog 1.9.2 REQUIRED)
message(STATUS "Found spdlog v${spdlog_VERSION}: ${spdlog_DIR}")

# zenoh-c
find_package(zenohc 1.9.0 REQUIRED)
message(STATUS "Found zenohc v${zenohc_VERSION}: ${zenohc_DIR}")

# zenoh-cpp
find_package(zenohcxx 1.9.0 REQUIRED)
message(STATUS "Found zenohcxx v${zenohcxx_VERSION}: ${zenohcxx_DIR}")

# RBDyn
set(Boost_USE_STATIC_LIBS OFF)
find_package(RBDyn 1.9.3 REQUIRED)
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
  zenohcxx::zenohc
  RBDyn::RBDyn
  RBDyn::Parsers
)

# C++20 required (PUBLIC so the standard propagates to consumers)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

# Use moderate compiler warning option
if(CMAKE_HOST_UNIX)
  target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)
else()
  target_compile_options(${PROJECT_NAME} PUBLIC /W1)
endif()

# ===================================================================
#      LIBRARY INSTALLATION
# ===================================================================
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()
