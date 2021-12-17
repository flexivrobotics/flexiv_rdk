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
  cartesian_impedance_control
  joint_impedance_control
  joint_position_control
  plan_execution
  primitive_execution
  robot_dynamics
  display_robot_states
  series_operation
  floating_with_soft_limits
  auto_recovery
  visualization
)

foreach(example ${EXAMPLE_LIST})
  # create executable from source
  add_executable(${example} example/${example}.cpp)

  # link dependencies
  target_include_directories(${example} 
    PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen
  )

  target_link_libraries(${example} 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libFlexivRdk.a
      anl
      Threads::Threads
  )
endforeach()


# ===================================
#      CONFIGURE ALL TESTS
# ===================================
# tests bin output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test) 

# list of tests
set(TEST_LIST
  test_dynamic_engine
  test_scheduler
  test_set_tool
  test_timeliness_monitor
  test_data_integrity
  test_endurance
  test_log
  test_loop_latency
)

foreach(test ${TEST_LIST})
  # create executable from source
  add_executable(${test} test/${test}.cpp)

  # link dependencies
  target_include_directories(${test} 
    PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen
  )

  target_link_libraries(${test} 
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/libFlexivRdk.a
      anl
      Threads::Threads
  )
endforeach()
