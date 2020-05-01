# Invariant EKF for IMU+GPS System
Implementation of an Invariant EKF for a system outfitted with an inertial measurement unit (IMU) and a GPS. We used the [Zurich Urban Micro Aerial Vehicle Dataset](http://rpg.ifi.uzh.ch/zurichmavdataset.html) to test our filter. For more details, please refer to our [technical report](https://github.com/ghaggin/invariant-ekf/blob/master/docs/report.pdf).

## Repository Contents
* docs
  - Project paper
* inekf
  - Invariant EKF implementation in C++
  - Modules and test cases
  - Dependency information and build instruction in ```inekf/Readme.md```
* scripts
  - Matlab scripts used for development
  - Fake data generation and tests
  - AGZ data set loading and testing
