#include <IEKF.hpp>
#include <utils.hpp>

#include <cmath>
#include <iostream>

using namespace std::chrono;
using std::cos;
using std::pow;
using std::sin;
using namespace Eigen;
using namespace iekf;

using Matrix5d = IEKF::Matrix5d;
using Matrix9d = IEKF::Matrix9d;
using Timestamp = IEKF::Timestamp;

void IEKF::prediction(
    const Vector3d& acc, const Vector3d& gyro, duration<double> dt_dur)
{
    const double dt = dt_dur.count();
    const double dt2 = dt * dt;

    const auto Rk = R();
    const auto pk = p();
    const auto vk = v();
    const auto& w = gyro;
    const auto& a = acc;

    auto g = (Vector3d() << 0, 0, -9.81).finished();

    Matrix3d Rk1 = Rk * gamma0(w * dt);
    Vector3d vk1 = vk + Rk * gamma1(w * dt) * a * dt + g * dt;
    Vector3d pk1 =
        pk + vk * dt + 0.5 * Rk * gamma2(w * dt) * a * dt2 + 0.5 * g * dt2;

    mu_.block<3, 3>(0, 0) = Rk1;
    mu_.block<3, 1>(0, 3) = vk1;
    mu_.block<3, 1>(0, 4) = pk1;
}

void IEKF::correction(const Vector3d& gps)
{
}

void IEKF::addImu(
    const Timestamp& timestamp, const Vector3d& acc, const Vector3d& gyro)
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
