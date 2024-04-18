cmake_minimum_required(VERSION 3.10)
project(planning)
#set(CMAKE_VERBOSE_MAKEFILE ON)

add_definitions(-DMODULE_NAME="${PROJECT_NAME}")

if (CROSS_BUILD_AARCH64)
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
    ${CMAKE_SOURCE_DIR}/src/planning
    ${CMAKE_SOURCE_DIR}/src/planning/navigation
    ${CMAKE_SOURCE_DIR}/src/planning/world_model
    ${CYBER_INCLUDE_DIR}/cyber
    ${CYBER_INCLUDE_DIR}/cyber/proto
    ${CYBER_INCLUDE_DIR}/neolix_common
)

link_directories(
    ${CYBER_LIB_DIR}
)

file(GLOB_RECURSE MATH_SRC_LIST 
    math/back_decision_model/*.cpp
    math/level_k/*.cpp
    math/common/*.cpp
    math/curve1d/*.cpp
    math/fempos/*.cpp
    math/frame_conversion/*.cpp
    math/piecewise_jerk/*.cpp
    math/public_math/*.cpp
    math/hpipm_solver/*.cpp
    math/ilqr_solver/*.cpp
    math/linear_curve_fit.cpp
    math/polygon_curve_fit.cpp
    math/curve_fit/*.cpp
    math/mcts/*.cpp
)

file(GLOB_RECURSE PLANNING_SRCS 
    common/*.cpp
    config/*.cpp
    planning_manager/*.cpp
    deciders/*.cpp
    navigation/*.cpp
    planning_map/*.cpp
    observer/*.cpp
    probe/*.cpp
    proxy/*.cpp
    scenario_manager/*.cpp
    sim_planner/*.cpp
    task/*.cpp
    traffic_law/*.cpp
    util/*.cpp
    world_model/*.cpp
)

file(GLOB_RECURSE REFERENCE_LINE_SRC_LIST
    reference_line/*.cpp
    reference_line/reference_line_generators/*.cpp
)

set(SRC_LIST
    planning_component.cpp
    ${MATH_SRC_LIST}
    ${PLANNING_SRCS}
    ${REFERENCE_LINE_SRC_LIST}
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
    hpipm
    blasfeo
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} planning_common) 
# install
install(TARGETS
    ${PROJECT_NAME}    
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
add_executable(cloud_navigation navigation/cloud_navigation/cloud_navigation.cpp)
target_link_libraries(
    cloud_navigation
    protobuf
    neolix_proto
    cyber
    cyberverse
    spdlog
    civetweb
    curl
    pthread
    ${PROJECT_NAME}

)
