cmake_minimum_required(VERSION 3.10)
project(modrob CXX)


include(CMakeHeader.cmake)

# ##############################################################################
# Examples
# ##############################################################################
# Add library and test case with link libraries
add_library(example SHARED src/example.cpp)
target_add_test_case(test_example test_example.cpp)
target_link_libraries(test_example example)

# Add a test case with no associated library and no link libraries
target_add_test_case(test_sophus test_sophus.cpp)
# ##############################################################################

# ##############################################################################
# IEKF
# ##############################################################################
add_library(iekf SHARED src/IEKF.cpp)
target_add_test_case(test_iekf test_iekf.cpp)
target_link_libraries(test_iekf iekf)
# ##############################################################################

# ##############################################################################
# utils
# ##############################################################################
target_add_test_case(test_utils test_utils.cpp)