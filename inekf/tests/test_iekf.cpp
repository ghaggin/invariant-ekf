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
using Matrix15d = IEKF::Matrix15d;
using Timestamp = IEKF::Timestamp;
using Seconds = IEKF::Seconds;

const double tol = 1e-5;

/*****************************************************************************/
// return absolute value of the max difference of any single element in two
// matrices
template <typename EigenMatd>
static double max_diff(const EigenMatd& m1, const EigenMatd& m2)
{
    return (m1 - m2).cwiseAbs().maxCoeff();
}

/*****************************************************************************/
// Defind some shorthand identity matrices
const Matrix3d Id3 = Matrix3d::Identity();
const Matrix5d Id5 = Matrix5d::Identity();
const Matrix15d Id15 = Matrix15d::Identity();

/*****************************************************************************/
BOOST_AUTO_TEST_CASE(default_constructor)
{
    IEKF iekf;
    Matrix5d mu;
    Matrix15d Sigma;
    Timestamp time;
    std::tie(mu, Sigma, time) = iekf.getState();

    auto a = (mu - Matrix5d::Identity()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (Sigma - Matrix15d::Identity()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);
}

/*****************************************************************************/
BOOST_AUTO_TEST_CASE(constructor)
{
    auto mu_in = arange_square<double, 5>();
    auto Sigma_in = arange_square<double, 15>();

    IEKF iekf(mu_in, Sigma_in);

    Matrix5d mu_out;
    Matrix15d Sigma_out;
    Timestamp time_out;
    std::tie(mu_out, Sigma_out, time_out) = iekf.getState();

    auto a = (mu_out - arange_square<double, 5>()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);

    a = (Sigma_out - arange_square<double, 15>()).cwiseAbs().maxCoeff();
    BOOST_CHECK_CLOSE(a, 0, tol);
}

/*****************************************************************************/
BOOST_AUTO_TEST_CASE(interfaces)
{
    auto mu = arange_square<double, 5>();
    auto Sigma = Matrix15d::Identity();
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

/*****************************************************************************/
// Run a test with zero inputs for accel and gyro, should return
// the starting state
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

/*****************************************************************************/
// Test with constant gyroscope measurements and accel just measureing gravity
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

/*****************************************************************************/
// Test with constant accel but no gyro input.  Body aligned with world frame
// should simulate pure rectilinear motion
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

    // Extract state
    auto R = iekf.R();
    auto p = iekf.p();
    auto v = iekf.v();

    // Set solution
    //  R - vehicle should not have rotated -> identity
    //  p - should move 1/2*a*t^2, since t = 1 and a = 1, p = 1/2
    //  v - should change to a*t, since a = 1 and t = 1, v = 1
    auto R_sol = Id3;
    auto p_sol = (Vector3d{} << 0.5, 0, 0).finished();
    auto v_sol = (Vector3d{} << 1., 0, 0).finished();

    // Check the state vs solution
    auto a = max_diff(R, R_sol);
    BOOST_CHECK(a < 1e-8);

    a = max_diff(p, p_sol);
    BOOST_CHECK(a < 1e-8);

    a = max_diff(v, v_sol);
    BOOST_CHECK(a < 1e-8);

    // Now accelerate in y for 1 sec
    // note that velocity in x is now 1
    acc = (Vector3d{} << 0, 1, g).finished();
    time = Timestamp{Seconds{2}};
    iekf.addImu(time, acc, gyro);
    R = iekf.R();
    p = iekf.p();
    v = iekf.v();
    p_sol = (Vector3d{} << 1.5, 0.5, 0).finished();
    v_sol = (Vector3d{} << 1., 1., 0).finished();
    BOOST_CHECK(max_diff(R, R_sol) < 1e-8);
    BOOST_CHECK(max_diff(p, p_sol) < 1e-8);
    BOOST_CHECK(max_diff(v, v_sol) < 1e-8);

    // Now accelerate in z for 1 sec
    // note that x and y velocty are now 1
    acc = (Vector3d{} << 0, 0, g + 1).finished();
    time = Timestamp{Seconds{3}};
    iekf.addImu(time, acc, gyro);
    R = iekf.R();
    p = iekf.p();
    v = iekf.v();
    p_sol = (Vector3d{} << 2.5, 1.5, 0.5).finished();
    v_sol = (Vector3d{} << 1., 1., 1.).finished();
    BOOST_CHECK(max_diff(R, R_sol) < 1e-8);
    BOOST_CHECK(max_diff(p, p_sol) < 1e-8);
    BOOST_CHECK(max_diff(v, v_sol) < 1e-8);
}

/*****************************************************************************/
// Test with constant accel and gyro over 1 sec
// Correct output determined from the prediction function
// in scripts/LIEKF.m with default initial conditions, dt = 1 s
// a = [1 2 3], w = [4 5 6]
BOOST_AUTO_TEST_CASE(test_accel_and_gyro)
{
    // Solution determined from scripts/LIEKF.m
    Matrix5d mu_sol =
        (Matrix5d{} << -0.422960948855940, 0.052841708771363, 0.904605875261157,
            1.686650850304059, 0.425996492893050, 0.880247438019417,
            -0.213015890828015, 0.424014950343734, 1.932585941427824,
            0.482297009692243, 0.215101100887779, 0.975618769842437,
            0.043583624539450, -7.211588851392560, -4.257578503338903, 0, 0, 0,
            1.000000000000000, 0, 0, 0, 0, 0, 1.000000000000000)
            .finished();

    Vector3d a{1, 2, 3};
    Vector3d w{4, 5, 6};
    Timestamp time{Seconds{1}};

    IEKF iekf{};

    iekf.addImu(time, a, w);

    Matrix5d mu = iekf.mu();

    BOOST_CHECK(max_diff(mu, mu_sol) < 1e-8);
}