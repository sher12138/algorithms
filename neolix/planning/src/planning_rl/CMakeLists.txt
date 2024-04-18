cmake_minimum_required(VERSION 3.10)
project(planning_rl)
# add_definitions(-DMODULE_NAME="${PROJECT_NAME}")
# cmake_policy(SET CMP0015 NEW)
include_directories(
  ${CMAKE_INSTALL_PREFIX}/include
  ${CMAKE_INSTALL_PREFIX}/include/cyber
  ${CMAKE_INSTALL_PREFIX}/include/cyber/proto
  ${CMAKE_INSTALL_PREFIX}/include/cyber/spdlog
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/common
)
message(STATUS "CMAKE_INSTALL_PREFIX:" ${CMAKE_INSTALL_PREFIX})
link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib
  ${CMAKE_INSTALL_PREFIX}/bin
)

# Build onboard shared library
file(GLOB_RECURSE SRCS 
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)
add_library(${PROJECT_NAME} SHARED  ${SRCS})
target_link_libraries(${PROJECT_NAME} 
  gflags
  glog
  jsoncpp
  protobuf
  neolix_proto
  cyberverse
  ${PROTOBUF_LIBRARIES}
  ${TENSORRT_LIBRARIES}
  ${CUDA_LIBRARIES}
  cyber
  stdc++fs
  spdlog
)

# Install
install(TARGETS ${PROJECT_NAME} 
        LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
        RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
# install(TARGETS ${PROJECT_NAME} 
#         LIBRARY DESTINATION "/home/caros/neolix/opt/lib"
#         RUNTIME DESTINATION "/home/caros/neolix/opt/bin")
