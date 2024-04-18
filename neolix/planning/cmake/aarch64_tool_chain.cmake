set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(TOOLCHAIN_PATH "/mnt")
set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
SET(CMAKE_FIND_ROOT_PATH
    ${CMAKE_SOURCE_DIR}/../gears/aarch64/include
    ${CMAKE_SOURCE_DIR}/../gears/aarch64/lib
    ${CMAKE_INSTALL_PREFIX}/inlcude
    ${CMAKE_INSTALL_PREFIX}/lib
    ${TOOLCHAIN_PATH}/usr/include/aarch64-linux-gnu
    ${TOOLCHAIN_PATH}/usr/lib/aarch64-linux-gnu
    ${TOOLCHAIN_PATH}/usr/local/include
    ${TOOLCHAIN_PATH}/usr/local/include/boost
    ${TOOLCHAIN_PATH}/usr/local/include/gflags
    ${TOOLCHAIN_PATH}/usr/local/include/glog
    ${TOOLCHAIN_PATH}/usr/local/include/gtest
    ${TOOLCHAIN_PATH}/usr/local/include/gmock
    ${TOOLCHAIN_PATH}/usr/local/include/opencv4
    ${TOOLCHAIN_PATH}/usr/local/include/Poco
    ${TOOLCHAIN_PATH}/usr/local/include/proj
    ${TOOLCHAIN_PATH}/usr/local/lib
    ${TOOLCHAIN_PATH}
)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

message(STATUS "Using tool chain file: ${CMAKE_TOOLCHAIN_FILE}")

option(CROSS_BUILD_AARCH64 "cross build aarch64" ON)
