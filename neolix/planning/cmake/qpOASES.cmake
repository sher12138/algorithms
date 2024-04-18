set(QP_INCLUDE_DIR "${GEARS_ARCH}/include/qpOASES")
set(QP_LINK_DIR "${GEARS_ARCH}/lib")
set(QP_LIBRARY "${GEARS_ARCH}/lib/libqpOASES.so")

include_directories(${QP_INCLUDE_DIR})
link_directories(${QP_LINK_DIR})
message(STATUS "QP_INCLUDE_DIR: ${QP_INCLUDE_DIR}")
message(STATUS "QP_LINK_DIR: ${QP_LINK_DIR}")
message(STATUS "QP_LIBRARY: ${QP_LIBRARY}")
