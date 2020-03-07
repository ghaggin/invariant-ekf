cmake_minimum_required(VERSION 3.10)
project(mobrob CXX)

include_directories(include)

add_library(example SHARED src/example.cpp)
set(TEST_LIB ${TEST_LIB} example)

option(BUILD_TESTS "Build the unit test modules" ON)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
set(CMAKE_BUILD_TYPE Debug)

# Testing ###
if(BUILD_TESTS)
  find_package(
    Boost
    COMPONENTS unit_test_framework
    REQUIRED)
  file(GLOB_RECURSE UnitTests_sources tests/*.cpp)
  add_executable(UnitTests ${UnitTests_sources})
  target_link_libraries(UnitTests ${TEST_LIB})
  target_link_libraries(UnitTests boost_unit_test_framework)
  enable_testing()
  add_test(UnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/UnitTests)
endif()

message(STATUS "")
message(STATUS "Building a ${CMAKE_BUILD_TYPE} build")
message(STATUS "")
