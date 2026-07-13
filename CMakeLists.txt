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

# Select the prebuilt library for this platform.
#
# On Linux and macOS the RDK is shipped as a self-contained SHARED library that
# statically embeds and symbol-hides all thirdparty dependencies except Eigen.
# Consumers therefore only need Eigen3 (+ Threads); the other dependencies are
# invisible and cannot collide with the consumer's own copies.
#
# The RDK is a self-contained shared library that statically embeds and
# symbol-hides its thirdparty dependencies (except Eigen), so consumers only
# need Eigen (+ Threads).
#
# On Windows the shared library is a DLL, and linking one needs two files: the
# import library (.lib, at link time) and the DLL itself (.dll, at runtime).
# RDK_RUNTIME_LIB names the .dll; RDK_LIB names the .lib. On all other platforms
# RDK_LIB is the single shared-library artifact and RDK_RUNTIME_LIB is empty.
message(STATUS "OS: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")
set(RDK_RUNTIME_LIB "")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(RDK_LIB "libflexiv_rdk.x86_64-linux-gnu.so")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(RDK_LIB "libflexiv_rdk.aarch64-linux-gnu.so")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64")
    set(RDK_LIB "libflexiv_rdk.arm64-darwin.dylib")
  else()
    message(FATAL_ERROR "macOS with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "AMD64")
    set(RDK_LIB "flexiv_rdk.win_amd64.lib")
    set(RDK_RUNTIME_LIB "flexiv_rdk.win_amd64.dll")
  else()
    message(FATAL_ERROR "Windows with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
else()
  message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} is currently not supported.")
endif()

# Download a release asset by file name, verifying its SHA256 against the
# checked-in <name>.sha256 placeholder.
function(rdk_download_release_asset asset_name)
  file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${asset_name}.sha256 expected_hash)
  set(asset_url https://github.com/flexivrobotics/flexiv_rdk/releases/download/v${PROJECT_VERSION}/${asset_name})
  message(STATUS "Downloading ${asset_url} ...")
  file(DOWNLOAD ${asset_url} ${CMAKE_CURRENT_BINARY_DIR}/${asset_name}
       SHOW_PROGRESS
       TLS_VERIFY ON
       EXPECTED_HASH SHA256=${expected_hash}
       STATUS status)
  list(GET status 0 code)
  if(NOT code EQUAL 0)
    message(FATAL_ERROR "Download failed: ${status}")
  endif()
  message(STATUS "Download finished: ${asset_name}")
endfunction()

# The link-time artifact (import .lib on Windows, the .so/.dylib/.a elsewhere).
rdk_download_release_asset(${RDK_LIB})
# On Windows also fetch the runtime DLL that the import library refers to.
if(RDK_RUNTIME_LIB)
  rdk_download_release_asset(${RDK_RUNTIME_LIB})
endif()

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================
# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
message(STATUS "Found Threads")

# Eigen3
# Header-only, used in the public flexiv::rdk API, so it is always required
# (and must stay ABI-compatible with the version the library was built against).
find_package(Eigen3 3.4.0 REQUIRED)
message(STATUS "Found Eigen3 v${Eigen3_VERSION}: ${Eigen3_DIR}")

# The remaining dependencies (spdlog, zenoh-c/zenoh-cpp, RBDyn, ...) are embedded
# and symbol-hidden inside the shared library, so consumers must NOT find or
# link them.

# ===================================================================
#      CREATE LIBRARY
# ===================================================================
# Create the library target with a dummy source. The dummy output is replaced
# at install time by the downloaded prebuilt shared library (see
# FlexivInstallLibrary).
add_library(${PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/dummy.cpp)

# Create an alias of the target with flexiv namespace
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to dependencies. Only Eigen3 + Threads are needed: Eigen leaks through
# the public API and Threads is required at link time. The other dependencies
# are embedded inside the shared library.
target_link_libraries(${PROJECT_NAME} PUBLIC
  Threads::Threads
  Eigen3::Eigen
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
