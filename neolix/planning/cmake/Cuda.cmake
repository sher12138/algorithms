message(STATUS "CMAKE_SYSTEM_PROCESSOR=" ${CMAKE_SYSTEM_PROCESSOR})
# find_package(CUDA REQUIRED)
set(CUDA_FOUND 1 CACHE INTERNAL "CUDA_FOUND")
set(CUDA_ROOT ${TOOLCHAIN_PATH}/usr/local/cuda ${TOOLCHAIN_PATH}/usr/local/cuda/targets/${CMAKE_SYSTEM_PROCESSOR}-linux)
set(CUDA_LIBRARY_DIRS ${TOOLCHAIN_PATH}/usr/local/cuda/lib64 ${TOOLCHAIN_PATH}/usr/local/cuda/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib)
include_directories(${TOOLCHAIN_PATH}/usr/include/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu ${TOOLCHAIN_PATH}/usr/local/cuda/targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include)
set(CMAKE_CUDA_FLAGS "-ccbin ${CMAKE_CXX_COMPILER} -Xcompiler -fPIC" CACHE STRING "" FORCE)
SET(CMAKE_CUDA_COMPILER ${TOOLCHAIN_PATH}/usr/local/cuda/bin/nvcc)


foreach(LIBDIR IN LISTS CUDA_LIBRARY_DIRS)
    message(STATUS "LIBDIR = ${LIBDIR}")
    file(GLOB LIBRARIES "${LIBDIR}/*.so")
    message(STATUS "LIBRARIES = ${LIBRARIES}")
    list(APPEND CUDA_LIBRARIES ${LIBRARIES})
endforeach()

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
message(STATUS "CUDA_INCLUDE_DIRS = ${CUDA_INCLUDE_DIRS}")
message(STATUS "CUDA_LIBRARY_DIRS = ${CUDA_LIBRARY_DIRS}")
message(STATUS "CUDA_LIBRARIES = ${CUDA_LIBRARIES}")
message(STATUS "CMAKE_CUDA_FLAGS = ${CMAKE_CUDA_FLAGS}")
message(STATUS "CMAKE_CUDA_COMPILER = ${CMAKE_CUDA_COMPILER}")
