cmake_minimum_required(VERSION 3.10)
project(navigation_test)

message(STATUS "CMAKE_SOURCE_DIR:" ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_CURRENT_BINARY_DIR:" ${CMAKE_CURRENT_BINARY_DIR})

include_directories(
  .
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/common
  ${CMAKE_SOURCE_DIR}/src/planning
  ${CMAKE_SOURCE_DIR}/src/planning/navigation
  ${CMAKE_SOURCE_DIR}/src/third_party
  ${CYBER_INCLUDE_DIR}/cyber
  ${CYBER_INCLUDE_DIR}/cyber/proto
  ${CYBER_INCLUDE_DIR}/neolix_common
)

link_directories(
  ${navigation_BINARY_DIR}
  ${GRPC_LIBRARY_DIRS}
  ${CYBER_LIB_DIR}
)

file(GLOB_RECURSE TEST_SRCS
  ./*.cpp
)

# Build all test files
set(TEST_SRC_LIST
  main.cpp
  ${TEST_SRCS}
)

# Build all test files
add_executable(${PROJECT_NAME} ${TEST_SRC_LIST})
add_dependencies(${PROJECT_NAME} planning_common planning)
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
  gtest
  gtest_main
  spdlog
  planning
  civetweb
  curl
)
