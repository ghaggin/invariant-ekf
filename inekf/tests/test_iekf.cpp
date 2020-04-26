#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_iekf
#include <boost/test/unit_test.hpp>

#include <IEKF.hpp>
#include <utils.hpp>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <tuple>

using namespace Eigen;
using namespace iekf;
using namespace std::chrono;

using Matrix5d = IEKF::Matrix5d;
using Matrix9d = IEKF::Matrix9d;
using Timestamp = IEKF::Timestamp;
using Seconds = IEKF::Seconds;

const double tol = 1e-5;

template <typename EigenMatd>
static double max_diff(const EigenMatd& m1, const EigenMatd& m2)
{
    return (m1 - m2).cwiseAbs().maxCoeff();
}

const Matrix3d Id3 = Matrix3d::Identity();
const Matrix5d Id5 = Matrix5d::Identity();
const Matrix9d Id9 = Matrix9d::Identity();

BOOST_AUTO_TEST_CASE(default_constructor)
{
    IEKF iekf;
    Matrix5d mu;
    Matrix9d Sigma;
    Timestamp time;
    std::tie(mu, Sigma, time) = iekf.getState();

    auto a = (mu - Matrix5d::Identity()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (Sigma - Matrix9d::Identity()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);
}

BOOST_AUTO_TEST_CASE(constructor)
{
    auto mu_in = arange_square<double, 5>();
    auto Sigma_in = arange_square<double, 9>();

    IEKF iekf(mu_in, Sigma_in);

    Matrix5d mu_out;
    Matrix9d Sigma_out;
    Timestamp time_out;
    std::tie(mu_out, Sigma_out, time_out) = iekf.getState();

    auto a = (mu_out - arange_square<double, 5>()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (Sigma_out - arange_square<double, 9>()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);
}

BOOST_AUTO_TEST_CASE(interfaces)
{
    auto mu = arange_square<double, 5>();
    auto Sigma = Matrix9d::Identity();
    IEKF iekf(mu, Sigma);
    auto R = iekf.R();
    auto p = iekf.p();
    auto v = iekf.v();

    auto R_sol = (Matrix3d() << 0, 1, 2, 5, 6, 7, 10, 11, 12).finished();
    auto p_sol = (Vector3d() << 4, 9, 14).finished();
    auto v_sol = (Vector3d() << 3, 8, 13).finished();

    auto a = (R - R_sol).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (p - p_sol).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (v - v_sol).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);
}

BOOST_AUTO_TEST_CASE(prediction_zero_meas)
{
    // Default construct with mu = Id
    IEKF iekf{};

    auto acc = Vector3d::Zero();
    auto gyro = Vector3d::Zero();
    Timestamp time{Seconds{0}};

    // Add imu measurement
    iekf.addImu(time, acc, gyro);

    auto mu = iekf.mu();

    auto a = max_diff(mu, Id5);
    BOOST_CHECK_CLOSE(a, 0, tol);
}

BOOST_AUTO_TEST_CASE(prediction_const_gyro)
{
    // Default construct with mu = Id
    IEKF iekf;

    auto g = iekf.g();

    auto acc = (Vector3d() << 0.0, 0.0, g).finished();
    auto gyro = (Vector3d() << 2 * M_PI, 0, 0).finished();

    Timestamp time{Seconds{1}};

    iekf.addImu(time, acc, gyro);

    auto R = iekf.R();
    auto R_sol = Id3;

    auto a = max_diff(R, R_sol);
    BOOST_CHECK(a < 1e-8);
}

BOOST_AUTO_TEST_CASE(prediction_const_acc)
{
    // Default construct with mu = Id
    IEKF iekf;

    auto g = iekf.g();

    // Measure 1 m/s^2 accel in body frame x
    // keep body frame aligned with world frame, look for accel in x
    auto acc = (Vector3d() << 1.0, 0.0, g).finished();
    auto gyro = Vector3d::Zero();

    // Default constructor sets time to zero
    // Set first imu measurement at 1 seconds
    Timestamp time{Seconds{1}};

    // Add imu measurement
    // Accelerate 1 m/s^2 in x for 1 sec
    iekf.addImu(time, acc, gyro);

    auto R = iekf.R();
    auto p = iekf.p();
    auto v = iekf.v();

    auto R_sol = Id3;
    auto p_sol = (Vector3d() << 0.5, 0, 0).finished();
    auto v_sol = (Vector3d() << 1., 0, 0).finished();

    auto a = max_diff(R, R_sol);
    BOOST_CHECK(a < 1e-8);

    a = max_diff(p, p_sol);
    BOOST_CHECK(a < 1e-8);
    std::cout << p << std::endl;

    a = max_diff(v, v_sol);
    BOOST_CHECK(a < 1e-8);
}