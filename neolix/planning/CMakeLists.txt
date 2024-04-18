cmake_minimum_required(VERSION 3.10)
project(planning)

cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0015 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -DBOOST_BIND_GLOBAL_PLACEHOLDERS")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O0 -Wall -g -ggdb")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -O0 -Wall -g -ggdb")
else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -O2")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w -O2")
endif()

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O0 -ggdb -mno-avx512f -mno-avx512ifma")
	#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -O0 -ggdb -mno-avx512f -mno-avx512ifma")
endif()

message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

include(${CMAKE_SOURCE_DIR}/cmake/Gears.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Eigen.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Cyberverse.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Boost.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/GLog.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/GFlags.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/GTest.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Protobuf.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ProtoCompiler.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Cyber.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/JsonCpp.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/OSQP.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/qpOASES.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Grpc.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Hpipm.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Blasfeo.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Cuda.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/Tensorrt.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/OpenCV.cmake)

set(BUILD_SRC OFF CACHE BOOL "build_src")
set(BUILD_UNIT_TEST OFF CACHE BOOL "build_unit_test")

message(STATUS "Build option:"
        " -> src:[${BUILD_SRC}];"
        " -> unit_test[${BUILD_UNIT_TEST}];")
include_directories(${CYBER_INCLUDE_DIR}/cyber/spdlog)

if(${BUILD_SRC})
	add_subdirectory(src/common)
	add_subdirectory(src/aeb)
	add_subdirectory(src/config_server)
        add_subdirectory(src/planning)
        #add_subdirectory(src/planning_rl)
endif(${BUILD_SRC})

if(${BUILD_UNIT_TEST})
        add_subdirectory(unit_test/aeb_test)
        add_subdirectory(unit_test/common_test)
        add_subdirectory(unit_test/planning_test)
        add_subdirectory(unit_test/world_model_test)
        add_subdirectory(unit_test/navigation_test)
        #add_subdirectory(unit_test/planning_rl_test)
endif(${BUILD_UNIT_TEST})

# install
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/launch)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/dag/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/dag)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/conf/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/conf)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/
        DESTINATION ${CMAKE_INSTALL_PREFIX}/scripts)
