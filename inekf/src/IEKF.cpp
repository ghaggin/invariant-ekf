#include <IEKF.hpp>

using namespace std::chrono;
using namespace Eigen;
using namespace iekf;

using Matrix5d = IEKF::Matrix5d;
using Matrix9d = IEKF::Matrix9d;
using Timestamp = IEKF::Timestamp;

void IEKF::prediction(
    const Vector3f& acc, const Vector3f& gyro, duration<double> dt)
{
}

void IEKF::correction(const Vector3d& gps)
{
}

void IEKF::addImu(
    const Timestamp& timestamp, const Vector3f& acc, const Vector3f& gyro)
{
    // Set the time to the current timestamp
    time_ = timestamp;

    // Calculate dt based on last observed time and current timestamp
    auto dt = timestamp - time_last_predict_;
    time_last_predict_ = timestamp;

    prediction(acc, gyro, dt);
    time_ = timestamp;
}

void IEKF::addGps(const Timestamp& timestamp, const Vector3d& gps)
{
    correction(gps);
    time_ = timestamp;
}

std::tuple<Matrix5d&, Matrix9d&, Timestamp&> IEKF::getState()
{
    return std::tie(mu_, Sigma_, time_);
}
