# C++ Implementation

## Adding Module
To create a new library, add source files to ```src``` and add headers to ```include```.  To build the library, add the following to the CMakeLists.txt
```
add_library(example SHARED src1.cpp src2.cpp scr3.cpp)
```
where src1.cpp, src2.cpp, src3.cpp are the source files for the library example.  To add a test case using the boost unit test frame work, create the test file in the tests directory.  To build and link the test case add the following to the CMakeLists.txt
```
target_add_test_case(<test-name> <test-src-files>)
target_link_libraries(<test-name> <test-link-libraries>)
```

## Dependencies
If you are building on linux, boost should be on your system and findable by cmake.  If you dont have boost run 

```
$ sudo apt install libboost-dev
```

For linear algebra and lie groups we can use Eigen and Sohpus respectively.  These are both header only libraries so installing them should be as simple as copying the headers to the desired location.  Typically this should be in ```/usr/local``` but feel free to put them anywhere that the compiler can find them.  Get additional help for [Eigen here](https://eigen.tuxfamily.org/dox/GettingStarted.html) and for [Sophus here](https://github.com/strasdat/Sophus).  For sophus you will need to clone the repo.  There are instructions for installing the dependencies and building the examples/tests in the bash script ```scripts/install_linux_deps.sh```.  I think you can probably ignore most of this and just copy the ```sophus``` folder in the main directory to your header install location, that is the only important thing that script does for us.

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