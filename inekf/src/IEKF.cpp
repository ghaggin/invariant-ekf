#include <IEKF.hpp>
#include <utils.hpp>

#include <eigen3/unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <iostream>

using namespace std::chrono;
using std::cos;
using std::pow;
using std::sin;
using namespace Eigen;
using namespace iekf;

using Matrix5d = IEKF::Matrix5d;
using Matrix15d = IEKF::Matrix15d;
using Timestamp = IEKF::Timestamp;

void IEKF::prediction(
    const Vector3d& acc, const Vector3d& gyro, duration<double> dt_dur)
{
    const double dt = dt_dur.count();
    const double dt2 = dt * dt;

    const Matrix3d Rk = R();
    const Vector3d pk = p();
    const Vector3d vk = v();
    const Vector3d& w = gyro;
    const Vector3d& a = acc;

    Vector3d g = (Vector3d() << 0, 0, -9.81).finished();

    Matrix3d Rk1 = Rk * gamma0(w * dt);
    Vector3d vk1 = vk + Rk * gamma1(w * dt) * a * dt + g * dt;
    Vector3d pk1 =
        pk + vk * dt + 0.5 * Rk * gamma2(w * dt) * a * dt2 + 0.5 * g * dt2;

    mu_.block<3, 3>(0, 0) = Rk1;
    mu_.block<3, 1>(0, 3) = vk1;
    mu_.block<3, 1>(0, 4) = pk1;

    Matrix15d A = Matrix15d::Zero();
    A.block<3, 3>(0, 0) = -skew(w);
    A.block<3, 3>(0, 9) = -Matrix3d::Identity();
    A.block<3, 3>(3, 0) = -skew(a);
    A.block<3, 3>(3, 3) = -skew(w);
    A.block<3, 3>(3, 12) = -Matrix3d::Identity();
    A.block<3, 3>(6, 3) = Matrix3d::Identity();
    A.block<3, 3>(6, 6) = -skew(w);

    // // Set covariance to the identity
    auto& Q = Matrix15d::Identity();

    auto& phi = (A * dt).exp();
    Sigma_ =
        phi * Sigma_ * (phi.transpose()) + phi * Q * (phi.transpose()) * dt;
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

std::tuple<Matrix5d&, Matrix15d&, Timestamp&> IEKF::getState()
{
    return std::tie(mu_, Sigma_, time_);
}
