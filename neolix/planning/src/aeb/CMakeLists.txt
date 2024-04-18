cmake_minimum_required(VERSION 3.10)
project(aeb)

# set(CMAKE_VERBOSE_MAKEFILE ON)
add_definitions(-DMODULE_NAME="${PROJECT_NAME}")

if(CROSS_BUILD_AARCH64)
    message("set cross build aarch64 include_directories and link_directories")
    include_directories(${CMAKE_INSTALL_PREFIX}/../opt/inlcude)
    include_directories(${CMAKE_INSTALL_PREFIX}/../gears/aarch64/include)
    include_directories(${TOOLCHAIN_PATH}/usr/include/aarch64-linux-gnu)
    include_directories(${TOOLCHAIN_PATH}/usr/local/include)
    link_directories(${CMAKE_INSTALL_PREFIX}/../opt/lib)
    link_directories(${CMAKE_INSTALL_PREFIX}/../gears/aarch64/lib)
    link_directories(${TOOLCHAIN_PATH}/usr/lib/aarch64-linux-gnu)
    link_directories(${TOOLCHAIN_PATH}/usr/local/lib)
endif()

# build
include_directories(
    .
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/aeb
    ${CYBER_INCLUDE_DIR}/cyber
    ${CYBER_INCLUDE_DIR}/cyber/proto
    ${CYBER_INCLUDE_DIR}/neolix_common
)

link_directories(
    ${CYBER_LIB_DIR}
)

file(GLOB_RECURSE AEB_SRCS
    common/*.cpp
    config/*.cpp
    util/*.cpp
    env/*.cpp
    util/*.cpp
    collision_check/*.cpp
)

set(SRC_LIST
    aeb_component.cpp
    aeb.cpp
    ${AEB_SRCS}
)

add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
target_link_libraries(
    ${PROJECT_NAME}
    neolix_proto
    cyber
    gflags
    glog
    jsoncpp
    osqp
    boost_system
    cyberverse
    neolix_common
    planning_common
    spdlog
)

# install
install(TARGETS
    ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)