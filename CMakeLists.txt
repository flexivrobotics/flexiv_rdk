cmake_minimum_required(VERSION 3.16.3)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_rdk VERSION 1.5.1)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo")

# Configure Python RDK
option(INSTALL_PYTHON_RDK "Install Python RDK (to user site packages path)" OFF)
if(INSTALL_PYTHON_RDK)
  if(NOT RDK_PYTHON_VERSION)
    set(RDK_PYTHON_VERSION 3.10 CACHE STRING "Python version to install RDK for" FORCE)
  endif()
  set_property(CACHE RDK_PYTHON_VERSION PROPERTY STRINGS "3.10" "3.12")
  find_package(Python3 ${RDK_PYTHON_VERSION} EXACT COMPONENTS Interpreter REQUIRED)
endif()

# Set C++ and Python library filename
message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(RDK_CPP_LIB "libflexiv_rdk.x86_64-linux-gnu.a")
    set(RDK_PY_LIB "flexivrdk.cpython-${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}-x86_64-linux-gnu.so")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(RDK_CPP_LIB "libflexiv_rdk.aarch64-linux-gnu.a")
    set(RDK_PY_LIB "flexivrdk.cpython-${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}-aarch64-linux-gnu.so")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64")
    set(RDK_CPP_LIB "libflexiv_rdk.arm64-darwin.a")
    set(RDK_PY_LIB "flexivrdk.cpython-${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}-darwin.so")
  else()
    message(FATAL_ERROR "Mac with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "AMD64")
    set(RDK_CPP_LIB "flexiv_rdk.win_amd64.lib")
    set(RDK_PY_LIB "flexivrdk.cp${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}-win_amd64.pyd")
  else()
    message(FATAL_ERROR "Windows with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
endif()

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================
# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
if(Threads_FOUND)
  message(STATUS "Found Threads: HAVE_PTHREAD = ${THREADS_HAVE_PTHREAD_ARG}")
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
#      INSTALL C++ LIBRARY
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
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${RDK_CPP_LIB}
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
message(STATUS "C++ RDK library: ${RDK_CPP_LIB}")
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()

# ===================================================================
#      INSTALL PYTHON LIBRARY
# ===================================================================
if(INSTALL_PYTHON_RDK)
  # Get user site packages path
  message(STATUS "Python executable: ${Python3_EXECUTABLE}")
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import site; print(site.getusersitepackages())"
    OUTPUT_VARIABLE USER_SITE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  message(STATUS "Python site packages path: ${USER_SITE}")

  # Install Python RDK to user site packages path
  message(STATUS "Python RDK library: ${RDK_PY_LIB}")
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/lib_py/${RDK_PY_LIB}" DESTINATION ${USER_SITE})
endif()
