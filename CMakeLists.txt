# use cmake 3 which supports targets
cmake_minimum_required(VERSION 3.1.3)

# ===================================
#      PROJECT SETUP
# ===================================
project(FlexivRdkExampleAndTest)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

option(BUILD_FOR_ARM64 "Link to RDK library for arm64 processor, otherwise link to x64" OFF)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ===================================
#      CONFIGURE ALL EXAMPLES
# ===================================
# examples bin output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/example) 

# list of examples
set(EXAMPLE_LIST
  auto_recovery
  cartesian_impedance_control
  clear_fault
  display_robot_states
  floating_with_soft_limits
  gripper_control
  joint_impedance_control
  joint_position_control
  plan_execution
  primitive_execution
  robot_dynamics
  series_operation
  visualization
)

foreach(example ${EXAMPLE_LIST})
  # create executable from source
  add_executable(${example} example/${example}.cpp)

  # link dependencies
  target_include_directories(${example} 
    PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen3
  )

  # Link basic libraries
  target_link_libraries(${example} 
      Threads::Threads
      anl
  )

  # Link arm64 or x64 version of libFlexivRdk
  if (${BUILD_FOR_ARM64})
    target_link_libraries(${example} 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/cpp/arm64/libFlexivRdk.a)
  else()
    target_link_libraries(${example} 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/cpp/x64/libFlexivRdk.a)
  endif()

endforeach()


# ===================================
#      CONFIGURE ALL TESTS
# ===================================
# tests bin output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test) 

# list of tests
set(TEST_LIST
  test_dynamics_engine
  test_dynamics_with_tool
  test_endurance
  test_log
  test_loop_latency
  test_scheduler
  test_timeliness_monitor
)

foreach(test ${TEST_LIST})
  # create executable from source
  add_executable(${test} test/${test}.cpp)

  # link dependencies
  target_include_directories(${test} 
    PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen3
  )

  # Link basic libraries
  target_link_libraries(${test} 
      Threads::Threads
      anl
  )

  # Link arm64 or x64 version of libFlexivRdk
  if (${BUILD_FOR_ARM64})
    target_link_libraries(${test} 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/cpp/arm64/libFlexivRdk.a)
  else()
    target_link_libraries(${test} 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/linux/cpp/x64/libFlexivRdk.a)
  endif()

endforeach()
