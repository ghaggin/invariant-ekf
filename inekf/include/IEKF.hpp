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
    // using Matrix15d = Eigen::Matrix<double, 9, 9>;
    using Matrix15d = Eigen::Matrix<double, 15, 15>;
    using Matrix3d = Eigen::Matrix<double, 3, 3>;
    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Timestamp = std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>;
    using Seconds = std::chrono::duration<double, std::ratio<1, 1>>;

public:
    IEKF() : mu_(Matrix5d::Identity()), Sigma_(Matrix15d::Identity())
    {
    }

    IEKF(const Matrix5d& mu, const Matrix15d& Sigma)
        : mu_(mu), Sigma_(Sigma), time_(Seconds(0)), time_last_predict_(time_)
    {
    }

    IEKF(const Matrix5d& mu, const Matrix15d& Sigma, const Timestamp& time)
        : mu_(mu), Sigma_(Sigma), time_(time), time_last_predict_(time_)
    {
    }

    void resetFilter(const Timestamp& time);

    // Add an IMU measurement to the filter
    void addImu(const Timestamp& timestamp, const Eigen::Vector3d& acc,
        const Eigen::Vector3d& gyro);

    // Add a gps measurement
    void addGps(const Timestamp& timestamp, const Eigen::Vector3d& gps);

    // Return the state and time stamp
    std::tuple<Matrix5d&, Matrix15d&, Timestamp&> getState();

    // Return rotatino matrix R
    Matrix3d R() const
    {
        return mu_.block(0, 0, 3, 3);
    }

    // Return position vector p
    Vector3d p() const
    {
        return mu_.block(0, 4, 3, 1);
    }

    // Return velocity vector v
    Vector3d v() const
    {
        return mu_.block(0, 3, 3, 1);
    }

    const Matrix5d& mu() const
    {
        return mu_;
    }

    double g() const
    {
        return g_;
    }

private:
    // Perform the prediction step of the filter
    void prediction(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
        std::chrono::duration<double> dt);

    // Perform the correction step of the filter
    void correction(const Eigen::Vector3d& gps);

    // State mean and covaraiance variables
    // and timestamp
    Matrix5d mu_;
    Matrix15d Sigma_;
    Timestamp time_;
    Timestamp time_last_predict_;

    const double g_ = 9.81;
};

}  // namespace iekf