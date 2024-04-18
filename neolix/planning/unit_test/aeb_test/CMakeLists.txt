cmake_minimum_required(VERSION 3.10)
project(aeb_test)

message(STATUS "CMAKE_SOURCE_DIR:" ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_CURRENT_BINARY_DIR:" ${CMAKE_CURRENT_BINARY_DIR})
message(STATUS "aeb_BINARY_DIR:" ${aeb_BINARY_DIR})

include_directories(
  .
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/common
  ${CMAKE_SOURCE_DIR}/src/aeb
  ${CMAKE_SOURCE_DIR}/src/third_party
  ${CYBER_INCLUDE_DIR}/cyber
  ${CYBER_INCLUDE_DIR}/cyber/proto
  ${CYBER_INCLUDE_DIR}/neolix_common
)

link_directories(
  ${aeb_BINARY_DIR}
  ${GRPC_LIBRARY_DIRS}
  ${CYBER_LIB_DIR}
)

file(GLOB_RECURSE TEST_SRCS
  config/*.cpp
  function_task/*.cpp
)

# Build all test files
set(TEST_SRC_LIST
   main.cpp
   ${TEST_SRCS}
)

# Build all test files
add_executable(${PROJECT_NAME} ${TEST_SRC_LIST})
add_dependencies(${PROJECT_NAME} planning_common aeb)
target_link_libraries(
  ${PROJECT_NAME}
  protobuf
  neolix_proto
  cyber
  gflags 
  glog 
  jsoncpp 
  osqp
  boost_system 
  cyberverse
  planning_common
  aeb
  gtest
  gtest_main
  spdlog
)

# gtest_discover_tests(${PROJECT_NAME})

# Install
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/data/planning/test)
