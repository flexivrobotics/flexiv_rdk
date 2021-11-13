# use cmake 3 which supports targets
cmake_minimum_required(VERSION 3.1.3)

# ===== CMAKE SETTINGS =====
project(FlexivRdkTools)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) 

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# list of examples
set(TOOL_LIST
  keyboard_move_tcp
)

# ===== CONFIGURE ALL TOOLS =====
foreach(tool ${TOOL_LIST})
  # create executable from source
  add_executable(${tool} ${tool}.cpp)

  # link dependencies
  target_include_directories(${tool} 
    PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/Eigen
  )

  target_link_libraries(${tool} 
      ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/libFlexivRdk.a
      anl
      Threads::Threads
  )
endforeach()

# ===== LINK THIRDPARTY IF NEEDED =====
# RBDyn
find_package(RBDyn REQUIRED)
target_link_libraries(keyboad_move_tcp
    RBDyn
    RBDynParsers
)
