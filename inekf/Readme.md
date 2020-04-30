# C++ Implementation

## Dependencies
If you are building on linux, boost should be on your system and findable by cmake.  If you dont have boost run 

```
$ sudo apt install libboost-dev
```

For linear algebra we can use Eigen.  This is a header only libraries so installing it should be as simple as copying the headers to the desired location.  Typically this should be in ```/usr/local``` but feel free to put them anywhere that the compiler can find them.  Get additional help for [Eigen here](https://eigen.tuxfamily.org/dox/GettingStarted.html).

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
$ make -j
```

To run the test cases run ```ctest``` from the build directory after building.  Test cases located in ```tests``` folder.

## Usage
An example of the usage is provided in ```iekf_example.cpp```.

Filter will initialize with orientation aligned with the world frame and zero position and velocity at the origin.

For safety, we have a reset filter function ```IEKF::resetFilter(time, origin_lla)``` which sets the filter time (based on the system using the filter) and the origin in lla coordinates.  The timestamp is used to automatically determine the ```dt``` between successive predictions and the origin is used to convert the lattitude-longitude-altitude (LLA) coordinates into an east-north-up (ENU) system.

To add an imu measurementd run ```IEKF::addImu(ts, accel, gyro)``` where ```accel``` is the body frame acceleration and ```gyro``` is the angular rate.  ```ts``` is the timestamp of the measurement, and ```dt``` in the filter is set by the difference between successive predictions.  Once an imu measurement is added, the filter will run the prediction step using the provided values.  If this is the first prediction, make sure that the timestamp will not cause a large filter ```dt``` (see ```resetFilter``` above).

To add a gps measurement, pass the LLA gps coordinates to ```addGPS```.  This will run the correction step of the filter using the gps measurement.  GPS coordinates are converted to ENU coordinates for the update.  The origin of the ENU system is either determined by the first gps measurement passed to the filter or by the user is ```resetFilter``` is called.