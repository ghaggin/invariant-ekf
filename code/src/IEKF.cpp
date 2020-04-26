#include <IEKF.hpp>

using namespace std::chrono;
using namespace Eigen;
using namespace iekf;

using Matrix5d = IEKF::Matrix5d;
using Matrix9d = IEKF::Matrix9d;
using Timestamp = IEKF::Timestamp;

IEKF::IEKF()
    : mu_(Matrix<double, 5, 5>::Identity()),
      Sigma_(Matrix<double, 9, 9>::Identity())
{
}

void IEKF::prediction(const Vector3f& acc, const Vector3f& gyro)
{
}

void IEKF::correction(const Vector3d& gps)
{
}

void IEKF::addImu(
    const Timestamp& timestamp, const Vector3f& acc, const Vector3f& gyro)
{
}

void IEKF::addGps(const Timestamp& timestamp, const Vector3d& gps)
{
}

std::tuple<Matrix5d&, Matrix9d&, Timestamp&> IEKF::getState()
{
    return std::tie(mu_, Sigma_, time);
}
