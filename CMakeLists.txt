cmake_minimum_required(VERSION 3.4)

# ===================================================================
#      PROJECT CONFIG
# ===================================================================
project(flexiv_rdk VERSION 0.6.0)

# C++14 required
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Configure processor platform
set(BUILD_PLATFORM "x86_64" CACHE STRING "Processor platform")
set_property(CACHE BUILD_PLATFORM PROPERTY STRINGS "x86_64" "arm64")

# Set static library
message("Building for system: ${CMAKE_SYSTEM_NAME}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${BUILD_PLATFORM} STREQUAL "x86_64")
    set(RDK_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/libFlexivRdk.x86_64-linux-gnu.a")
  elseif(${BUILD_PLATFORM} STREQUAL "arm64")
    set(RDK_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/libFlexivRdk.aarch64-linux-gnu.a")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  if(${BUILD_PLATFORM} STREQUAL "x86_64")
    message(FATAL_ERROR "Mac with x86_64 processor is currently not supported.")
  elseif(${BUILD_PLATFORM} STREQUAL "arm64")
    set(RDK_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/libFlexivRdk.arm64-darwin.a")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(${BUILD_PLATFORM} STREQUAL "x86_64")
    set(RDK_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/FlexivRdk.x86_64-windows.lib")
  elseif(${BUILD_PLATFORM} STREQUAL "arm64")
    message(FATAL_ERROR "Windows with arm64 processor is currently not supported.")
  endif()
endif()

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================
# pthread
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ===================================================================
#      PROJECT LIBRARIES
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
  ${RDK_STATIC_LIBRARY}
  Threads::Threads
)

# Use moderate compiler warning option 
if(MSVC)
  target_compile_options(${PROJECT_NAME} INTERFACE /W1)
else()
  target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra)
endif()

# Install the INTERFACE library
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()

# Also install Eigen headers
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen3/Eigen
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} 
)
