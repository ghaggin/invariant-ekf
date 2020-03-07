# C++ Implementation

## Adding Module
To create a new library, add source files to ```src``` and add headers to ```include```.  To build the library, add the following to the CMakeLists.txt
```
add_library(example SHARED src1.cpp src2.cpp scr3.cpp)
```
where src1.cpp, src2.cpp, src3.cpp are the source files for the library example.  To add a test case using the boost unit test frame work, create the test file in the tests directory.  To link the libary to the test, add the following to the CMakeLists.txt
```
set(TEST_LIBS ${TEST_LIBS} example)
```

## Dependencies
If you are building on linux, boost should be on your system and findable by cmake.  If you dont have boost run 

```
$ sudo apt install libboost-dev
```

Additional dependencies should listed here as they are added.

## Building
Build project using the cmake build system.  Cmake version is set to 3.10 which is available on Ubuntu 18.04 through apt.

```
$ sudo apt install cmake
```

To build the project, create a build directory, run cmake, then make.
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

To run the test cases run ```ctest``` from the build directory after building.