cmake_minimum_required(VERSION 3.22.1)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_rdk VERSION 2.2 LANGUAGES CXX)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")

# Select the prebuilt library for this platform. RDK is shipped as a self-contained shared library.
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
    # On Windows the shared library is a DLL, and linking one needs two files: the import library (.lib, at link time)
    # and the DLL itself (.dll, at runtime). RDK_RUNTIME_LIB names the .dll; RDK_LIB names the .lib. On all other 
    # platforms RDK_LIB is the single shared-library artifact and RDK_RUNTIME_LIB is empty.
    set(RDK_LIB "flexiv_rdk.win_amd64.lib")
    set(RDK_RUNTIME_LIB "flexiv_rdk.win_amd64.dll")
  else()
    message(FATAL_ERROR "Windows with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
else()
  message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} is currently not supported.")
endif()

# Additionally verify the downloaded prebuilt library against the release's GPG-signed checksum
# list (SHA256SUMS + SHA256SUMS.asc), using the release signing key committed in this repository.
# Requires gpg. See SECURITY.md and RELEASING.md.
option(FLEXIV_RDK_VERIFY_SIGNATURE "Verify the GPG signature of the downloaded release assets" OFF)
# Public key and fingerprint of the Flexiv RDK release signing key.
# TODO: placeholders until the release signing key is published, see cmake/flexiv_release_key.asc.
set(FLEXIV_RDK_RELEASE_KEY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/flexiv_release_key.asc)
set(FLEXIV_RDK_RELEASE_KEY_FINGERPRINT "TODO_PUBLISH_RELEASE_KEY_FINGERPRINT")

# All assets of this version are downloaded from its GitHub release
set(RDK_RELEASE_URL https://github.com/flexivrobotics/flexiv_rdk/releases/download/v${PROJECT_VERSION})

# Download a file of the release into the build directory. If [expected_hash] is not empty, the
# download is verified against it.
function(rdk_download_file asset_name expected_hash)
  set(asset_url ${RDK_RELEASE_URL}/${asset_name})
  set(hash_args "")
  if(expected_hash)
    set(hash_args EXPECTED_HASH SHA256=${expected_hash})
  endif()
  message(STATUS "Downloading ${asset_url} ...")
  file(DOWNLOAD ${asset_url} ${CMAKE_CURRENT_BINARY_DIR}/${asset_name}
       SHOW_PROGRESS
       TLS_VERIFY ON
       ${hash_args}
       STATUS status)
  list(GET status 0 code)
  if(NOT code EQUAL 0)
    message(FATAL_ERROR "Download failed: ${status}")
  endif()
  message(STATUS "Download finished: ${asset_name}")
endfunction()

# Download a release asset by file name, verifying its SHA256 against the checked-in
# <name>.sha256 placeholder. The expected hash is also returned in [hash_var], so that the install
# step can check the file again right before installing it (see FlexivInstallLibrary).
function(rdk_download_release_asset asset_name hash_var)
  file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${asset_name}.sha256 expected_hash)
  string(STRIP "${expected_hash}" expected_hash)
  string(TOLOWER "${expected_hash}" expected_hash)
  string(LENGTH "${expected_hash}" hash_length)
  if(NOT expected_hash MATCHES "^[0-9a-f]+$" OR NOT hash_length EQUAL 64)
    message(FATAL_ERROR "lib/${asset_name}.sha256 does not contain a valid SHA256")
  endif()
  rdk_download_file(${asset_name} ${expected_hash})
  set(${hash_var} ${expected_hash} PARENT_SCOPE)
endfunction()

# The link-time artifact (import .lib on Windows, the .so/.dylib/.a elsewhere).
rdk_download_release_asset(${RDK_LIB} RDK_LIB_SHA256)
set(RDK_DOWNLOADED_ASSETS ${CMAKE_CURRENT_BINARY_DIR}/${RDK_LIB})
# On Windows also fetch the runtime DLL that the import library refers to.
if(RDK_RUNTIME_LIB)
  rdk_download_release_asset(${RDK_RUNTIME_LIB} RDK_RUNTIME_LIB_SHA256)
  list(APPEND RDK_DOWNLOADED_ASSETS ${CMAKE_CURRENT_BINARY_DIR}/${RDK_RUNTIME_LIB})
endif()

# Check the signed checksum list of the same release. Its files are downloaded without an expected
# hash because the GPG signature is what authenticates them.
if(FLEXIV_RDK_VERIFY_SIGNATURE)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivVerifyReleaseSignature.cmake)
  flexiv_check_release_key(${FLEXIV_RDK_RELEASE_KEY_FILE} "${FLEXIV_RDK_RELEASE_KEY_FINGERPRINT}" _)
  rdk_download_file(SHA256SUMS "")
  rdk_download_file(SHA256SUMS.asc "")
  flexiv_verify_release_signature(
    SUMS_FILE ${CMAKE_CURRENT_BINARY_DIR}/SHA256SUMS
    SIGNATURE_FILE ${CMAKE_CURRENT_BINARY_DIR}/SHA256SUMS.asc
    KEY_FILE ${FLEXIV_RDK_RELEASE_KEY_FILE}
    FINGERPRINT ${FLEXIV_RDK_RELEASE_KEY_FINGERPRINT}
    WORK_DIR ${CMAKE_CURRENT_BINARY_DIR}/release_signature
    ARTIFACTS ${RDK_DOWNLOADED_ASSETS})
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

# ===================================================================
#      CREATE LIBRARY
# ===================================================================
# Create the library target with a dummy source. The dummy output is replaced at
# install time by the downloaded prebuilt shared library (see FlexivInstallLibrary).
add_library(${PROJECT_NAME} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/lib/dummy.cpp)

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
)

# C++20 required
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
