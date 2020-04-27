# Build options and variables
option(BUILD_TESTS "Build the unit test modules" ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-g3 -Wall -Wconversion -Wextra -pedantic")
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_FLAGS "-g3 -Wall -Wextra -Wconversion --pedantic")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Set include directory
include_directories(include)

if(BUILD_TESTS)
  enable_testing()
endif()

function(target_add_test_case TEST_NAME TEST_SOURCE)
  if(BUILD_TESTS)
    add_executable(${TEST_NAME} tests/${TEST_SOURCE})
    add_test(${TEST_NAME} bin/${TEST_NAME})
    target_link_libraries(${TEST_NAME} boost_unit_test_framework)
  endif()
endfunction()

# Ouput build type (Release/Debug)
message(STATUS "")
message(STATUS "Building a ${CMAKE_BUILD_TYPE} build")
message(STATUS "")
