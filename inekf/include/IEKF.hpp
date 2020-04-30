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
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix15d = Eigen::Matrix<double, 15, 15>;
    using Matrix3d = Eigen::Matrix<double, 3, 3>;
    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Timestamp = std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>;
    using Seconds = std::chrono::duration<double, std::ratio<1, 1>>;

public:
    IEKF()
        : mu_(Matrix5d::Identity()),
          Sigma_(Matrix15d::Identity()),
          bias_(Vector6d::Zero())
    {
    }

    IEKF(const Matrix5d& mu, const Matrix15d& Sigma)
        : mu_(mu),
          Sigma_(Sigma),
          time_(Seconds(0)),
          time_last_predict_(time_),
          bias_(Vector6d::Zero())
    {
    }

    IEKF(const Matrix5d& mu, const Matrix15d& Sigma, const Timestamp& time)
        : mu_(mu),
          Sigma_(Sigma),
          time_(time),
          time_last_predict_(time_),
          bias_(Vector6d::Zero())
    {
    }

    void resetFilter(const Timestamp& time, const Vector3d& origin_lla);

    // Add an IMU measurement to the filter
    void addImu(const Timestamp& timestamp, const Eigen::Vector3d& acc,
        const Eigen::Vector3d& gyro);

    // Add a gps measurement in LLA form
    void addGps(const Timestamp& timestamp, const Eigen::Vector3d& gps);

    void set_origin(Eigen::Vector3d gps_lla)
    {
        origin_ = gps_lla;
        origin_set_ = true;
    }

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
    Vector6d bias_;
    bool origin_set_ = false;
    Vector3d origin_;  ///< origin coordinates in lla

    const double g_ = 9.81;
};

}  // namespace iekf