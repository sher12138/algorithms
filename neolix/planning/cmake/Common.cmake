set(COMMON_ROOT "${CMAKE_INSTALL_PREFIX}" CACHE PATH "COMMON_ROOT")
set(COMMON_INCLUDE_DIR "${COMMON_ROOT}/include/common")
set(COMMON_LIB_DIR "${COMMON_ROOT}/lib")

include_directories(${COMMON_INCLUDE_DIR}/kinematics)
include_directories(${COMMON_INCLUDE_DIR}/math)
include_directories(${COMMON_INCLUDE_DIR}/proto)
link_directories(${COMMON_LIB_DIR})
