cmake_minimum_required(VERSION 3.10)
project(planning_rl_test)
set(CYBER_LIB_DIR "/home/caros/neolix/opt/lib")
set(GRPC_LIBRARY_DIRS "/home/caros/neolix/gears/x86_64/lib")
message(STATUS "CMAKE_SOURCE_DIR:" ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_CURRENT_BINARY_DIR:" ${CMAKE_CURRENT_BINARY_DIR})
message(STATUS "CYBER_INCLUDE_DIR:" ${CYBER_INCLUDE_DIR})
include_directories(
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/planning_rl
  ${CMAKE_SOURCE_DIR}/src/planning_rl/common
  ${CYBER_INCLUDE_DIR}/cyber
  ${CYBER_INCLUDE_DIR}/cyber/proto
  ${CYBER_INCLUDE_DIR}/neolix_common
)
message(STATUS "GRPC_LIBRARY_DIRS:" ${GRPC_LIBRARY_DIRS})
message(STATUS "CYBER_LIB_DIR:" ${CYBER_LIB_DIR})
link_directories(
  ${GRPC_LIBRARY_DIRS}
  ${CYBER_LIB_DIR}
)

file(GLOB_RECURSE TEST_SRCS
  config/*.cpp
)

# Build all test files
set(TEST_SRC_LIST
   main.cpp
   ${TEST_SRCS}
)

# Build all test files
add_executable(${PROJECT_NAME} ${TEST_SRC_LIST})
add_dependencies(${PROJECT_NAME} planning_rl)
target_link_libraries(
  ${PROJECT_NAME}
  gflags
  glog
  spdlog
  jsoncpp
  protobuf
  neolix_proto
  cyberverse
  ${PROTOBUF_LIBRARIES}
  # ${TORCH_LIBRARIES}
  ${TENSORRT_LIBRARIES}
  cyber
  stdc++fs
  planning_rl
  gtest
  gtest_main
  libjsoncpp.a
)

# gtest_discover_tests(${PROJECT_NAME})

# Install
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data/
#  DESTINATION ${CMAKE_INSTALL_PREFIX}/data/planning/test)
