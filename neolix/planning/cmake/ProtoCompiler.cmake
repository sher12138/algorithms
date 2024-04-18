# ================================================================================
# Protobuf Compiler Module
# --------------------------------------------------------------------------------
#
# Defines the following functions
#
# 1. PROTOBUF_COMPILE_DIRECTORY
# 2. PROTOBUF_COMPILE_DIRECTORY_FLAT
# 3. PROTOBUF_COMPILE_DIRECTORY_FLAT_WITH_GRPC

################################
## PROTOBUF_COMPILE_DIRECTORY ##
################################
#
# Runs protoc to compile all the .proto files within the specified
# directory (search is done recursively) to corresponding headers and
# sources, making them CMake targets that can be depend on.
#
# The generated headers and sources are placed under
# ${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX}, with the
# original directory structure. See example for details on "keeping
# the original directory structure".
#
# ------------------------------------------------------------
# Arguments:
#
#   PROTO_DIR: the directory that contains the .proto files to be
#     compiled.
#
#   TARGET_PREFIX: the generated headers and sources are placed under
#     ${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX}.
#
#   PROTO_SRCS [OUTPUT]: The list of the paths of all the generated
#     sources.
#
# ------------------------------------------------------------
# Example:
#
#   protobuf_compile_directory(
#     ${CMAKE_CURRENT_SOURCE_DIR}/"proto"
#     "adu/common/proto"
#     MY_PROTO_SRCS)
#   add_executable(my_proto_lib ${MY_PROTO_SRCS})
#
# In the above example, all the .proto files under the directory
# "${CMAKE_CURRENT_SOURCE_DIR}/proto" will be compiled and the
# generated files will be placed under
# "${CMAKE_CURRENT_BINARY_DIR}/generated/adu/common/proto".
#
# The directory structure will be kept. This means that, a .proto file
# with path "${CMAKE_CURRENT_SOURCE_DIR}/proto/a/b/c.proto" will be
# compiled into
#
#    ${CMAKE_CURRENT_BINARY_DIR}/generated/adu/common/proto/a/b/c.pb.h
#    ${CMAKE_CURRENT_BINARY_DIR}/generated/adu/common/proto/a/b/c.pb.cc
#
# The good thing about this is that with
#
#    include_directories(SYSTEM ${CMAKE_CURRENT_BINARY_DIR}/generated)
#
# All the generated headers can be referred to with
#
# #include "adu/common/proto/..."
function(PROTOBUF_COMPILE_DIRECTORY PROTO_DIR TARGET_PREFIX PROTO_SRCS)
  # Search for all the .proto files recursively.
  file(GLOB_RECURSE PROTO_FILES RELATIVE ${PROTO_DIR} "${PROTO_DIR}/*.proto")

  set(TARGET_ROOT "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX}")

  foreach(PROTO_FILE ${PROTO_FILES})
    set(TMP_TARGET "${TARGET_ROOT}/${PROTO_FILE}")
    # Replace the trailing .proto to .pb.h and .pb.cc to get the
    # actual generated header and source file path.
    string(REGEX REPLACE ".proto$" ".pb.h" PROTO_FILE_PB_H "${TMP_TARGET}")
    string(REGEX REPLACE ".proto$" ".pb.cc" PROTO_FILE_PB_CC "${TMP_TARGET}")

    # Collect the path of the generated source file into the output.
    list(APPEND PROTO_SRC_LIST ${PROTO_FILE_PB_CC})
    # Get the actual directory path of the generated files so that we
    # can create them before generateding the .pb.h/.pb.cc files.
    get_filename_component(TARGET_DIRECTORY ${PROTO_FILE_PB_H} DIRECTORY)
    add_custom_command(
      OUTPUT
      "${PROTO_FILE_PB_H}"
      "${PROTO_FILE_PB_CC}"
      COMMAND ${CMAKE_COMMAND} -E make_directory ${TARGET_DIRECTORY}
      # see --cpp_out and -I in the help of protoc for more details.
      COMMAND "LD_LIBRARY_PATH=${PROTOBUF_LIBRARY_DIR}" ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out "${TARGET_ROOT}" -I "${PROTO_DIR}"
      "${PROTO_DIR}/${PROTO_FILE}"
      DEPENDS "${PROTO_DIR}/${PROTO_FILE}"
      VERBATIM)
  endforeach()
  # Handling the exposure of ${PROTO_SRCS} and force including the
  # generated directory.
  set(${PROTO_SRCS} ${PROTO_SRC_LIST} PARENT_SCOPE)
  include_directories(SYSTEM "${CMAKE_CURRENT_BINARY_DIR}/generated")
endfunction()

#####################################
## PROTOBUF_COMPILE_DIRECTORY_FLAT ##
#####################################
# NOTE(breakds): Compatible mode compile instruction for
#                adu/common/proto.
#
# Similar to PROTOBUF_COMPILE_DIRECTORY, runs protoc to compile all
# the .proto files within the specified directory (search is done
# recursively) to corresponding headers and sources, making them CMake
# targets that can be depend on.
#
# The generated headers and sources are placed under
# ${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX} with a flat
# structure.
#
# ------------------------------------------------------------
# Arguments:
#
#   PROTO_DIR: the directory that contains the .proto files to be
#     compiled.
#
#   TARGET_PREFIX: the generated headers and sources are placed under
#     ${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX}.
#
#   PROTO_SRCS [OUTPUT]: The list of the paths of all the generated
#     sources.
#
# ------------------------------------------------------------
# Example:
#
#   protobuf_compile_directory_flat(
#     "${CMAKE_CURRENT_SOURCE_DIR}/proto"
#     "adu/common/proto"
#     MY_PROTO_SRCS)
#   add_executable(my_proto_lib ${MY_PROTO_SRCS})
#
# In the above example, all the .proto files under the directory
# "${CMAKE_CURRENT_SOURCE_DIR}/proto" will be compiled and the
# generated files will be placed under
# "${CMAKE_CURRENT_BINARY_DIR}/generated/adu/common/proto".
#
# The directory structure will be DISCARDED and FORCED FLAT. This
# means that, a .proto file with path
# "${CMAKE_CURRENT_SOURCE_DIR}/proto/a/b/name.proto" will be compiled
# into
#
#    ${CMAKE_CURRENT_BINARY_DIR}/generated/adu/common/proto/name.pb.h
#    ${CMAKE_CURRENT_BINARY_DIR}/generated/adu/common/proto/name.pb.cc
function(PROTOBUF_COMPILE_DIRECTORY_FLAT PROTO_DIR TARGET_PREFIX PROTO_SRCS)
  # Search for all the .proto files recursively.
  file(GLOB_RECURSE PROTO_FILES RELATIVE ${PROTO_DIR} "${PROTO_DIR}/*.proto")
  set(TARGET_ROOT "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX}")

  # Generate all the include paths.
  foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(INCLUDE_DIR "${PROTO_DIR}/${PROTO_FILE}" DIRECTORY)
    list(FIND _proto_include_paths ${INCLUDE_DIR} _contains_already)
    if(${_contains_already} EQUAL -1)
      list(APPEND _proto_include_paths -I ${INCLUDE_DIR})
    endif()
  endforeach()

  foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_FILE_NAME ${PROTO_FILE} NAME)
    set(TMP_TARGET "${TARGET_ROOT}/${PROTO_FILE_NAME}")
    string(REGEX REPLACE ".proto$" ".pb.h" PROTO_FILE_PB_H "${TMP_TARGET}")
    string(REGEX REPLACE ".proto$" ".pb.cc" PROTO_FILE_PB_CC "${TMP_TARGET}")

    list(APPEND PROTO_SRC_LIST ${PROTO_FILE_PB_CC})

    get_filename_component(TARGET_DIRECTORY ${PROTO_FILE_PB_H} DIRECTORY)
    add_custom_command(
      OUTPUT
      "${PROTO_FILE_PB_H}"
      "${PROTO_FILE_PB_CC}"
      COMMAND ${CMAKE_COMMAND} -E make_directory ${TARGET_DIRECTORY}
      # see --cpp_out and -I in the help of protoc for more details.
      COMMAND "LD_LIBRARY_PATH=${PROTOBUF_LIBRARY_DIR}" ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out "${TARGET_DIRECTORY}" ${_proto_include_paths}
      "${PROTO_DIR}/${PROTO_FILE}"
      DEPENDS "${PROTO_DIR}/${PROTO_FILE}"
      VERBATIM)
  endforeach()
  set(${PROTO_SRCS} ${PROTO_SRC_LIST} PARENT_SCOPE)
  include_directories(SYSTEM "${CMAKE_CURRENT_BINARY_DIR}/generated")
endfunction()

###############################################
## PROTOBUF_COMPILE_DIRECTORY_FLAT_WITH_GRPC ##
###############################################
# NOTE(breakds): Compatible mode compile instruction for
#                adu/common/proto.
#
# This is mostly the same as PROTOBUF_COMPILE_DIRECTORY_FLAT (see
# above). In fact, it does everything that
# PROTOBUF_COMPILE_DIRECTORY_FLAT does, and in addition it will
# compile specified .proto to .grpc.* files with the GRPC plugin.
#
# Requires GRPC package.
function(PROTOBUF_COMPILE_DIRECTORY_FLAT_WITH_GRPC PROTO_DIR GRPC_PROTOS
    TARGET_PREFIX PROTO_SRCS)
  # Search for all the .proto files recursively.
  file(GLOB_RECURSE PROTO_FILES RELATIVE ${PROTO_DIR} "${PROTO_DIR}/*.proto")
  set(TARGET_ROOT "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX}")

  # Generate all the include paths.
  foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(INCLUDE_DIR "${PROTO_DIR}/${PROTO_FILE}" DIRECTORY)
    list(FIND _proto_include_paths ${INCLUDE_DIR} _contains_already)
    if(${_contains_already} EQUAL -1)
      list(APPEND _proto_include_paths -I ${INCLUDE_DIR})
    endif()
  endforeach()

  foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_FILE_NAME ${PROTO_FILE} NAME)
    set(TMP_TARGET "${TARGET_ROOT}/${PROTO_FILE_NAME}")
    string(REGEX REPLACE ".proto$" ".pb.h" PROTO_FILE_PB_H "${TMP_TARGET}")
    string(REGEX REPLACE ".proto$" ".pb.cc" PROTO_FILE_PB_CC "${TMP_TARGET}")

    list(APPEND PROTO_SRC_LIST ${PROTO_FILE_PB_CC})

    get_filename_component(TARGET_DIRECTORY ${PROTO_FILE_PB_H} DIRECTORY)
    add_custom_command(
      OUTPUT
      "${PROTO_FILE_PB_H}"
      "${PROTO_FILE_PB_CC}"
      COMMAND ${CMAKE_COMMAND} -E make_directory ${TARGET_DIRECTORY}
      # see --cpp_out and -I in the help of protoc for more details.
      COMMAND "LD_LIBRARY_PATH=${PROTOBUF_LIBRARY_DIR}" ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out "${TARGET_DIRECTORY}" ${_proto_include_paths}
      "${PROTO_DIR}/${PROTO_FILE}"
      DEPENDS "${PROTO_DIR}/${PROTO_FILE}"
      VERBATIM)
  endforeach()

  # Handle GRPC generations.
  foreach(PROTO_FILE ${GRPC_PROTOS})
    get_filename_component(PROTO_FILE_NAME ${PROTO_FILE} NAME)
    set(TMP_TARGET "${TARGET_ROOT}/${PROTO_FILE_NAME}")
    string(REGEX REPLACE ".proto$" ".grpc.pb.h" PROTO_FILE_GRPC_PB_H "${TMP_TARGET}")
    string(REGEX REPLACE ".proto$" ".grpc.pb.cc" PROTO_FILE_GRPC_PB_CC "${TMP_TARGET}")

    list(APPEND PROTO_SRC_LIST ${PROTO_FILE_GRPC_PB_CC})

    get_filename_component(TARGET_DIRECTORY ${PROTO_FILE_PB_H} DIRECTORY)
    add_custom_command(
      OUTPUT
      "${PROTO_FILE_GRPC_PB_H}"
      "${PROTO_FILE_GRPC_PB_CC}"
      COMMAND ${CMAKE_COMMAND} -E make_directory ${TARGET_DIRECTORY}
      # see --cpp_out and -I in the help of protoc for more details.
      COMMAND "LD_LIBRARY_PATH=${PROTOBUF_LIBRARY_DIR}" ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS
      --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXECUTABLE}
      --grpc_out "${TARGET_DIRECTORY}" ${_proto_include_paths}
      "${PROTO_DIR}/${PROTO_FILE}"
      DEPENDS "${PROTO_DIR}/${PROTO_FILE}"
      VERBATIM)
  endforeach()

  set(${PROTO_SRCS} ${PROTO_SRC_LIST} PARENT_SCOPE)
  include_directories(SYSTEM "${CMAKE_CURRENT_BINARY_DIR}/generated")
endfunction()

# my added func to build with protos from cyber
function(PROTOBUF_COMPILE_DIRECTORY_FLAT_WITH_DIR PROTO_DIR TARGET_PREFIX PROTO_SRCS DIR)
  # Search for all the .proto files recursively.
  file(GLOB_RECURSE PROTO_FILES RELATIVE ${PROTO_DIR} "${PROTO_DIR}/*.proto")
  set(TARGET_ROOT "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX}")

  # Generate all the include paths.
  foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(INCLUDE_DIR "${PROTO_DIR}/${PROTO_FILE}" DIRECTORY)
    list(FIND _proto_include_paths ${INCLUDE_DIR} _contains_already)
    if(${_contains_already} EQUAL -1)
      list(APPEND _proto_include_paths -I ${INCLUDE_DIR})
    endif()
  endforeach()

  # Search for all the other .proto files recursively.
  file(GLOB_RECURSE OTHER_PROTO_FILES RELATIVE ${DIR} "${DIR}/*.proto")

  # Generate all the include paths.
  foreach(OTHER_PROTO_FILE ${OTHER_PROTO_FILES})
    get_filename_component(INCLUDE_DIR "${DIR}/${OTHER_PROTO_FILE}" DIRECTORY)
    list(FIND _proto_include_paths ${INCLUDE_DIR} _contains_already)
    if(${_contains_already} EQUAL -1)
      list(APPEND _proto_include_paths -I ${INCLUDE_DIR})
    endif()
  endforeach()

  foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_FILE_NAME ${PROTO_FILE} NAME)
    set(TMP_TARGET "${TARGET_ROOT}/${PROTO_FILE_NAME}")
    string(REGEX REPLACE ".proto$" ".pb.h" PROTO_FILE_PB_H "${TMP_TARGET}")
    string(REGEX REPLACE ".proto$" ".pb.cc" PROTO_FILE_PB_CC "${TMP_TARGET}")

    list(APPEND PROTO_SRC_LIST ${PROTO_FILE_PB_CC})

    get_filename_component(TARGET_DIRECTORY ${PROTO_FILE_PB_H} DIRECTORY)
    add_custom_command(
      OUTPUT
      "${PROTO_FILE_PB_H}"
      "${PROTO_FILE_PB_CC}"
      COMMAND ${CMAKE_COMMAND} -E make_directory ${TARGET_DIRECTORY}
      # see --cpp_out and -I in the help of protoc for more details.
      COMMAND "LD_LIBRARY_PATH=${PROTOBUF_LIBRARY_DIR}" ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out "${TARGET_DIRECTORY}" ${_proto_include_paths}
      "${PROTO_DIR}/${PROTO_FILE}"
      DEPENDS "${PROTO_DIR}/${PROTO_FILE}"
      VERBATIM)
  endforeach()
  set(${PROTO_SRCS} ${PROTO_SRC_LIST} PARENT_SCOPE)
  include_directories(SYSTEM "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_PREFIX}")
endfunction()

