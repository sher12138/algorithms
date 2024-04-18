cmake_minimum_required(VERSION 3.10)
project(planning_test)

message(STATUS "CMAKE_SOURCE_DIR:" ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_CURRENT_BINARY_DIR:" ${CMAKE_CURRENT_BINARY_DIR})

include_directories(
  .
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/common
  ${CMAKE_SOURCE_DIR}/src/planning
  ${CMAKE_SOURCE_DIR}/src/planning/world_model
  ${CMAKE_SOURCE_DIR}/src/third_party
  ${CYBER_INCLUDE_DIR}/cyber
  ${CYBER_INCLUDE_DIR}/cyber/proto
  ${CYBER_INCLUDE_DIR}/neolix_common
)

link_directories(
  ${planning_BINARY_DIR}
  ${GRPC_LIBRARY_DIRS}
  ${CYBER_LIB_DIR}
)

file(GLOB_RECURSE TEST_SRCS
  common/*.cpp
  config/*.cpp
  deciders/*.cpp
  math/common/*.cpp
  math/curve1d/*.cpp
  math/frame_conversion/*.cpp
  math/public_math/*.cpp
  math/ilqr_solver/*.cpp
  planning_manager/*.cpp
  observer/*.cpp
  proxy/*.cpp
  #reference_line/*.cpp
  scenario_manager/*.cpp
  state_machine/*.cpp
  util/*.cpp
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
  neolix_common
  cyber
  gflags
  glog
  jsoncpp
  osqp
  boost_system
  cyberverse
  planning_common
  planning
  gtest
  gtest_main
  spdlog
  fastrtps
  civetweb
  curl
)

# gtest_discover_tests(${PROJECT_NAME})

# Install
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/
DESTINATION ${CMAKE_INSTALL_PREFIX}/data/planning/test)
