#pragma once

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <tuple>

namespace iekf
{
class IEKF
{
public:
    // Define a bunch of commonly used types to save typing
    // and make the files more clear. These are consistent
    // with Eigen naming conventions.
    using Matrix5d = Eigen::Matrix<double, 5, 5>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix15d = Eigen::Matrix<double, 15, 15>;
    using Matrix3d = Eigen::Matrix<double, 3, 3>;
    using Vector3d = Eigen::Matrix<double, 3, 1>;

    // Shorthand chrono types
    using Timestamp = std::chrono::time_point<std::chrono::system_clock,
        std::chrono::duration<double>>;
    using Seconds = std::chrono::duration<double, std::ratio<1, 1>>;

public:
    // Default constructor initialize mu and Sigma to identity
    IEKF()
        : mu_(Matrix5d::Identity()),
          Sigma_(Matrix15d::Identity()),
          time_(Seconds(0)),
          time_last_predict_(time_),
          bias_(Vector6d::Zero())
    {
    }

    // Provide mu and Sigma, time initialized to zero
    IEKF(const Matrix5d& mu, const Matrix15d& Sigma)
        : mu_(mu),
          Sigma_(Sigma),
          time_(Seconds(0)),
          time_last_predict_(time_),
          bias_(Vector6d::Zero())
    {
    }

    // Provide mu Sigma and time
    IEKF(const Matrix5d& mu, const Matrix15d& Sigma, const Timestamp& time)
        : mu_(mu),
          Sigma_(Sigma),
          time_(time),
          time_last_predict_(time_),
          bias_(Vector6d::Zero())
    {
    }

    // Reset time and origin of filter
    void resetFilter(const Timestamp& time, const Vector3d& origin_lla);

    // Add an IMU measurement to the filter
    void addImu(const Timestamp& timestamp, const Eigen::Vector3d& acc,
        const Eigen::Vector3d& gyro);

    // Add a gps measurement in LLA form
    void addGps(const Timestamp& timestamp, const Eigen::Vector3d& gps);

    // Set the origin
    // bool is set to indicate to other
    // parts of the program that the origin
    // is set and does not need to be modified
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

    // Return the state mu
    const Matrix5d& mu() const
    {
        return mu_;
    }

    // Return the covariance Sigma
    const Matrix15d& Sigma() const
    {
        return Sigma_;
    }

    // Return the gravitational constant
    // used by the filter (could vary based
    // on location)
    double g() const
    {
        return g_;
    }

private:
    // Perform the prediction step of the filter
    void prediction(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
        std::chrono::duration<double> dt);

    // Perform the correction step of the filter using gps coordinates
    void correction(const Eigen::Vector3d& gps_lla);

    // State mean and covaraiance variables
    // and timestamp
    Matrix5d mu_;                  ///< state mean
    Matrix15d Sigma_;              ///< state covariance
    Timestamp time_;               ///< filter time
    Timestamp time_last_predict_;  ///< saved time of last predict step
    Timestamp time_last_gps_;      ///< saved time for last gps addition
    Vector6d bias_;                ///< imu bias vector (omega, accel)
    Vector3d origin_;              ///< origin coordinates in lla
    bool origin_set_ = false;      ///< indicates if gps origin set

    const double g_ = 9.81;  ///< gravitational constant
};

}  // namespace iekf