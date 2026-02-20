# QNX toolchain for cross-compilation of aarch64le ("le" - little endian) target

# Set target system
set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_PROCESSOR aarch64le)
set(arch gcc_ntoaarch64le)

# specify the cross compiler
set(CMAKE_C_COMPILER qcc)
set(CMAKE_C_COMPILER_TARGET ${arch})
set(CMAKE_CXX_COMPILER q++)
set(CMAKE_CXX_COMPILER_TARGET ${arch})

# Set qnx build flags
add_definitions("-D_QNX_SOURCE")

# Set QNX_TARGET
if (DEFINED ENV{QNX_TARGET})
    set (QNX_TARGET $ENV{QNX_TARGET})
else ()
    message (FATAL_ERROR "NO QNX TARGET ENV DEFINED")
endif ()

# Set the sysroot to the QNX target directory, so the compiler can find the correct headers and libraries for the target system.
set(CMAKE_SYSROOT ${QNX_TARGET})

# Set default cmake_install_prefix
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OR NOT DEFINED CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_PREFIX ${QNX_TARGET}/${CMAKE_SYSTEM_PROCESSOR}/usr/local CACHE PATH "cmake install prefix" FORCE)
endif()

# We want to search:
# 1. ${CMAKE_PREFIX_PATH}
# 2. ${CMAKE_INSTALL_PREFIX}
# 3. $ENV{QNX_TARGET}/${CMAKE_SYSTEM_PROCESSOR}/usr/local
# 4. $ENV{QNX_TARGET}/${CMAKE_SYSTEM_PROCESSOR}/usr
# 5. $ENV{QNX_TARGET}/${CMAKE_SYSTEM_PROCESSOR}
# 6. $ENV{QNX_TARGET}/usr
#
# To accomplish 3-6, append CMAKE_FIND_ROOT_PATH with:
# - $ENV{QNX_TARGET}/${CMAKE_SYSTEM_PROCESSOR}
# - $ENV{QNX_TARGET}/usr
# 
# However, CMAKE_FIND_ROOT_PATH will also be prepended to CMAKE_PREFIX_PATH and CMAKE_INSTALL_PREFIX.
# In order to search these without the prefix,
# append CMAKE_FIND_ROOT_PATH with:
# - CMAKE_PREFIX_PATH
# - CMAKE_INSTALL_PREFIX
#
# For details, see Confluence page: "Intro to CMake find_package()"
if (DEFINED CMAKE_PREFIX_PATH)
    list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_PREFIX_PATH})
endif()
list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_INSTALL_PREFIX})
list(APPEND CMAKE_FIND_ROOT_PATH ${QNX_TARGET}/${CMAKE_SYSTEM_PROCESSOR} ${QNX_TARGET}/usr)

# Remove duplicates, since we are appending
list(REMOVE_DUPLICATES CMAKE_FIND_ROOT_PATH)

# Search only the paths prefixed with CMAKE_FIND_ROOT_PATH
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# QNX does not support -pthread compiler flag,
# because pthread is already part of the c library
# However, FindThreads.cmake incorrectly sets CMAKE_HAVE_LIBC_PTHREAD OFF
# See: https://gitlab.kitware.com/cmake/cmake/-/issues/21579
set(CMAKE_HAVE_LIBC_PTHREAD ON)
