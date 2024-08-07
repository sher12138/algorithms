set(BLASFEO_FOUND 1 CACHE INTERNAL "BLASFEO_FOUND")
set(BLASFEO_ROOT "${GEARS_ARCH}" CACHE PATH "BLASFEO_ROOT")
set(BLASFEO_INCLUDE_DIRS ${BLASFEO_ROOT}/include/blasfeo/include CACHE INTERNAL "BLASFEO_INCLUDE_DIRS")
set(BLASFEO_LIBRARY_DIRS ${BLASFEO_ROOT}/lib CACHE INTERNAL "BLASFEO_LIBRARY_DIRS")
file(GLOB LIBRARIES "${BLASFEO_LIBRARY_DIRS}/libblasfeo*")
set(BLASFEO_LIBRARIES ${LIBRARIES} CACHE INTERNAL "BLASFEO_LIBRARIES")

include_directories(${BLASFEO_INCLUDE_DIRS})
link_directories(${BLASFEO_LIBRARY_DIRS})
message(STATUS "BLASFEO_INCLUDE_DIRS: ${BLASFEO_INCLUDE_DIRS}")
message(STATUS "BLASFEO_LIBRARY_DIRS: ${BLASFEO_LIBRARY_DIRS}")
message(STATUS "BLASFEO_LIBRARIES: ${BLASFEO_LIBRARIES}")