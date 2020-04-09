#pragma once

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <tuple>

namespace iekf
{
class IEKF
{
public:
    using Matrix5d = Eigen::Matrix<double, 5, 5>;
    using Matrix9d = Eigen::Matrix<double, 9, 9>;
    using Timestamp = std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>;

public:
    IEKF();

    // Add an IMU measurement to the filter
    void addImu(const Timestamp& timestamp, const Eigen::Vector3f& acc,
        const Eigen::Vector3f& gyro);
    void addGps(const Timestamp& timestamp, const Eigen::Vector3d& gps);
    std::tuple<Matrix5d&, Matrix9d&, Timestamp&> getState();

private:
    void prediction(const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

    void correction(const Eigen::Vector3d& gps);

    // State mean and covaraiance
    Matrix5d mu_;
    Matrix9d Sigma_;
    Timestamp time;
};

}  // namespace iekf