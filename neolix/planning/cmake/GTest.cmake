find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIR})

add_custom_target(check
        COMMAND ${CMAKE_CTEST_COMMAND})
enable_testing()

function(CREATE_TEST testName)
  message(STATUS "Creating test ${testName}")
  add_executable(test_${testName}
                 EXCLUDE_FROM_ALL
                 test_${testName}.cc)
  target_link_libraries(test_${testName}
                        # ${GTEST_BOTH_LIBRARIES}
                        gtest
                        gtest_main
                        ${ARGN}
                        ${GLOG_LIBRARIES})
  add_test(NAME test/${PROJECT_NAME}/test_${testName}
           COMMAND test_${testName})
  set_tests_properties(test/${PROJECT_NAME}/test_${testName}
                       PROPERTIES
                       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  add_dependencies(check test_${testName})
endfunction()
